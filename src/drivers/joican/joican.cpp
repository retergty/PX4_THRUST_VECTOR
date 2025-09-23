#include "joican.hpp"
#include <px4_platform_common/log.h>
using namespace joican;

extern "C" __EXPORT int joican_main(int argc, char *argv[])
{
	return Joican::main(argc, argv);
}
int Joican::task_spawn(int argc, char *argv[])
{
	Joican *instance = new Joican();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int Joican::init()
{
	if (_can1.initCan(0) < 0) {
		PX4_ERR("can1 init fail");
		return -1;
	}

	if (_can2.initCan(1) < 0) {
		PX4_ERR("can2 init fail");
		return -1;
	}

	if (_can1.start() < 0) {
		PX4_ERR("can1 start fail");
		return -1;
	};

	if (_can2.start() < 0) {
		PX4_ERR("can2 start fail");
		return -1;
	};

	int32_t Joican_enable = 1;

	(void)param_get(param_find("JOICAN_ENABLE"), &Joican_enable);

	if (Joican_enable > 0) {
		PX4_INFO("joican enable");
	}

	// enable servo
	ServoEnable();
	// send zero position
	sendServoSetpoint();

	return 0;
}
int Joican::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving damiao joint can.
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("joint_can", "driver");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int Joican::custom_command(int argc, char* argv[])
{
  if (!is_running())
  {
    print_usage("not running");
    return 1;
  }
  if (!strcmp(argv[0], "test"))
  {
    if (argc > 1)
    {
      if (!strcmp(argv[1], "can1"))
      {
	if (argc > 2)
	{
	  if (!strcmp(argv[2], "-s"))
	  {
	    if (argc > 3)
	    {
	      long servo = strtol(argv[3], nullptr, 10);
	      if (servo < 1 || servo > 4)
	      {
		PX4_ERR("servo %ld,not support", servo);
		return 0;
	      }
	      if (argc > 4)
	      {
		float angle = strtof(argv[4], nullptr);
		if (angle > 180.f)
		{
		  angle = 180.f;
		}
		else if (angle < -180.f)
		{
		  angle = -180.f;
		}
		PX4_INFO("send joint can command to %ld servo (1 ~ 4), %f angle", servo, (double)angle);
		Joican* joi_instance = get_instance();
		joi_instance->_can1_servo_posctl[servo - 1].setAbsPosition(angle);
		joi_instance->_can1_last_ret[servo - 1] =
		    joi_instance->_can1.send(joi_instance->_can1_servo_posctl[servo - 1]);
	      }
	      else
	      {
		PX4_ERR("missing argument");
	      }
	    }
	    else
	    {
	      PX4_ERR("missing argument");
	    }
	  }
	  else
	  {
	    PX4_ERR("argument %s unsupported.", argv[2]);
	  }
	}
	else
	{
	  PX4_ERR("missing argument");
	}
      }
      else if (!strcmp(argv[1], "can2"))
      {
	if (argc > 2)
	{
	  if (!strcmp(argv[2], "-s"))
	  {
	    if (argc > 3)
	    {
	      long servo = strtol(argv[3], nullptr, 10);
	      if (servo < 1 || servo > 4)
	      {
		PX4_ERR("servo %ld,not support", servo);
		return 0;
	      }
	      if (argc > 4)
	      {
		float angle = strtof(argv[4], nullptr);
		if (angle > 180.f)
		{
		  angle = 180.f;
		}
		else if (angle < -180.f)
		{
		  angle = -180.f;
		}
		PX4_INFO("send joint can command to %ld servo (1 ~ 4), %f angle", servo, (double)angle);
		Joican* joi_instance = get_instance();
		joi_instance->_can2_servo_posctl[servo - 1].setAbsPosition(angle);
		joi_instance->_can2_last_ret[servo - 1] =
		    joi_instance->_can2.send(joi_instance->_can1_servo_posctl[servo - 1]);
	      }
	      else
	      {
		PX4_ERR("missing argument");
	      }
	    }
	    else
	    {
	      PX4_ERR("missing argument");
	    }
	  }
	  else
	  {
	    PX4_ERR("argument %s unsupported.", argv[2]);
	  }
	}
	else
	{
	  PX4_ERR("missing argument");
	}
      }
      else
      {
	PX4_ERR("argument %s unsupported.", argv[1]);
	return 1;
      }
      return 0;
    }
    else
    {
      PX4_ERR("missing argument");
    }
  }

  return print_usage("unknown command");
}
int Joican::print_status()
{
  perf_print_counter(_cycle_perf);
  perf_print_counter(_interval_perf);
  _mixing_output.printStatus();
  _can1.print_status();
  _can2.print_status();
  PX4_INFO("last can1 send ret: %d %d %d %d", _can1_last_ret[0], _can1_last_ret[1], _can1_last_ret[2],
	   _can1_last_ret[3]);
  PX4_INFO("last can2 send ret: %d %d %d %d", _can2_last_ret[0], _can2_last_ret[1], _can2_last_ret[2],
	   _can2_last_ret[3]);

  PX4_INFO("can1 count secess: %d, fail: %d", _can1_sucess_count, _can1_fail_count);
  PX4_INFO("can2 count secess: %d, fail: %d", _can2_sucess_count, _can2_fail_count);

  PX4_INFO("number of frames got: %d", _get_count);
  for (int i = 0; i < 4; ++i)
  {
    PX4_INFO("can1 servo position: %f", (double)_can1_servo[i].position);
    PX4_INFO("can2 servo position: %f", (double)_can2_servo[i].position);
  }
  return 0;
}
void Joican::updateParams()
{
  OutputModuleInterface::updateParams();
}
void Joican::setCountCorRet(const int16_t ret, int& sucess_count, int& fail_count)
{
  if (ret == 0)
    sucess_count++;
  else
    fail_count++;
}
void Joican::sendServoSetpoint()
{
  //_can1_last_ret[0] = _can1.send(_can1_servo_posctl[0]);
  //_can2_last_ret[0] = _can2.send(_can2_servo_posctl[0]);

  if(_can1_servo[1].enable)
  {
  	_can1_last_ret[1] = _can1.send(_can1_servo_posctl[1]);
  }
  //_can2_last_ret[1] = _can2.send(_can2_servo_posctl[1]);

  //_can1_last_ret[2] = _can1.send(_can1_servo_posctl[2]);
  //_can2_last_ret[2] = _can2.send(_can2_servo_posctl[2]);

  //_can1_last_ret[3] = _can1.send(_can1_servo_posctl[3]);
  //   _can2_last_ret[3] = _can2.send(_can2_servo_posctl[3]);

  //_can1_last_ret[0] = _can1.send(_can1_servo_posctl, 4);
  for (int i = 0; i < 4; ++i)
  {
    setCountCorRet(_can1_last_ret[1], _can1_sucess_count, _can1_fail_count);
    // setCountCorRet(_can1_last_ret[1], _can2_sucess_count, _can2_fail_count);
  }
}
void Joican::Run()
{
  if (should_exit())
  {
    ScheduleClear();
    _mixing_output.unregister();

    exit_and_cleanup();
    return;
  }

  if (!_is_init)
  {
    init();
    _is_init = true;
  }

  perf_begin(_cycle_perf);
  perf_count(_interval_perf);

  _mixing_output.update();

  // check for parameter updates
  if (_parameter_update_sub.updated())
  {
    // clear update
    parameter_update_s pupdate;
    _parameter_update_sub.copy(&pupdate);

    // update parameters from storage
    updateParams();
  }

  // check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
  _mixing_output.updateSubscriptions(true);

  can_frame frame;
  while (_can1.receive(frame))
  {
    handReceiveFrame(frame, 1);
    _get_count++;
  }
  while (_can2.receive(frame))
  {
    handReceiveFrame(frame, 2);
    _get_count++;
  }

  perf_end(_cycle_perf);
}
bool Joican::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated)
{
  _count++;
  for (int i = 0; i < 4; ++i)
  {
    if (!_mixing_output.isFunctionSet(i))
    {
      // do not run any signal on disabled channels
      outputs[i] = 0;
    }
    if (!_mixing_output.isFunctionSet(i + 4))
    {
      // do not run any signal on disabled channels
      outputs[i + 4] = 0;
    }
    _can1_servo_posctl[i].setAbsPosition((float)outputs[i] / 500.f - 1.f + _can1_servo[i].zero_offset);	     // [-1, 1]
    _can2_servo_posctl[i].setAbsPosition((float)outputs[i + 4] / 500.f - 1.f + _can2_servo[i].zero_offset);  // [-1, 1]
  }

  //   if ((_count % 800) == 0)
  //   {
  //        sendServoSetpoint();
  //   }
  sendServoSetpoint();

  return true;
}
void Joican::ServoEnable()
{
  //ServoEnableFrame servo1{ 0x1 };
  ServoEnableFrame servo2{ 0x2 };
  //ServoEnableFrame servo3{ 0x3 };
  //ServoEnableFrame servo4{ 0x4 };

  //_can1.send(servo1);
  //_can2.send(servo1);

  _can1.send(servo2);
  //_can2.send(servo2);

  //_can1.send(servo3);
  //_can2.send(servo3);

  //_can1.send(servo4);
  //_can2.send(servo4);
}
void Joican::ServoDisable()
{
  ServoDisableFrame servo1{ 0x1 };
  ServoDisableFrame servo2{ 0x2 };
  ServoDisableFrame servo3{ 0x3 };
  ServoDisableFrame servo4{ 0x4 };

  _can1.send(servo1);
  _can2.send(servo1);

  _can1.send(servo2);
  _can2.send(servo2);

  _can1.send(servo3);
  _can2.send(servo3);

  _can1.send(servo4);
  _can2.send(servo4);
}
void Joican::handReceiveFrame(const can_frame& frame, const uint8_t can_instance)
{
  ServoState* servo = nullptr;
  uint8_t dev_id = frame.data[0];
  if (can_instance == 1)
  {
    servo = &_can1_servo[dev_id-1];
  }
  else if (can_instance == 2)
  {
    servo = &_can2_servo[dev_id-1];
  }
  else
  {
    return;
  }

  if (!servo->enable)
  {
    servo->enable = true;
  }

  uint16_t p_raw = (frame.data[1] << 8) | frame.data[2];
  //uint16_t v_raw = (frame.data[3] << 4) | (frame.data[4] >> 4);
  //uint16_t t_raw = ((frame.data[4] & 0xF) << 8) | frame.data[5];

  // only convert position because we only need position
  servo->position = AbsPostionCtlFrame::positionDecoder(p_raw);
}

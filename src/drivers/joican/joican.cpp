#include "joican.hpp"
#include <px4_platform_common/log.h>
using namespace joican;

extern "C" __EXPORT int joican_main(int argc, char* argv[])
{
  return Joican::main(argc, argv);
}
int Joican::task_spawn(int argc, char* argv[])
{
  Joican* instance = new Joican();

  if (!instance)
  {
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
  if (_can1.initCan(0) < 0)
  {
    PX4_ERR("can1 init fail");
    return -1;
  }

  if (_can2.initCan(1) < 0)
  {
    PX4_ERR("can2 init fail");
    return -1;
  }
  if (_can1.start() < 0)
  {
    PX4_ERR("can1 start fail");
    return -1;
  };
  if (_can2.start() < 0)
  {
    PX4_ERR("can2 start fail");
    return -1;
  };

  int32_t Joican_enable = 1;
  (void)param_get(param_find("JOICAN_ENABLE"), &Joican_enable);

  if (Joican_enable > 0)
  {
    PX4_INFO("joican enable");
  }

  // send zero position
  sendServoSetpoint();

  return 0;
}
int Joican::print_usage(const char* reason)
{
  if (reason)
  {
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
  _can1_last_ret[0] = _can1.send(_can1_servo_posctl[0]);
  _can2_last_ret[0] = _can2.send(_can2_servo_posctl[0]);

  _can1_last_ret[1] = _can1.send(_can1_servo_posctl[1]);
  _can2_last_ret[1] = _can2.send(_can2_servo_posctl[1]);

  _can1_last_ret[2] = _can1.send(_can1_servo_posctl[2]);
  _can2_last_ret[2] = _can2.send(_can2_servo_posctl[2]);

  _can1_last_ret[3] = _can1.send(_can1_servo_posctl[3]);
  _can2_last_ret[3] = _can2.send(_can2_servo_posctl[3]);

  for (int i = 0; i < 4; ++i)
  {
    setCountCorRet(_can1_last_ret[0], _can1_sucess_count, _can1_fail_count);
    setCountCorRet(_can1_last_ret[1], _can2_sucess_count, _can2_fail_count);
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
    _can1_servo_posctl[i].setAbsPosition((float)outputs[i] / 500.f - 1.f);	// [-1, 1]
    _can2_servo_posctl[i].setAbsPosition((float)outputs[i + 4] / 500.f - 1.f);	// [-1, 1]
  }

  if ((_count % 800) == 0)
  {
    // sendServoSetpoint();
  }
  sendServoSetpoint();
  return true;
}

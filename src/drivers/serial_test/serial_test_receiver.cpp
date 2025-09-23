#include "serial_test.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

void SerialTestReceiver::changeReadState(const uint8_t get_ch, const uint8_t des_ch,
                                         const SerialTestReceiver::ReadState right_to,
                                         const SerialTestReceiver::ReadState fail_to)
{
  if (get_ch == des_ch)
  {
    _read_state = right_to;
    _read_success++;
  }
  else
  {
    _read_state = fail_to;
    _read_error++;
  }
}

// 0: not yet
// 1: sucess
// -1: fail
int SerialTestReceiver::handleReceive(uint8_t* buf, int length)
{
  int read = 0;
  while (read < length)
  {
    uint8_t ch = buf[read++];
    switch (_read_state)
    {
      // sync 1
      case ReadState::NoSync:
        changeReadState(ch, 0x11, ReadState::Sync1, ReadState::NoSync);
        break;
      case ReadState::Sync1:
        changeReadState(ch, 0x22, ReadState::Sync2, ReadState::NoSync);
        break;
      case ReadState::Sync2:
        changeReadState(ch, 0x33, ReadState::Sync3, ReadState::NoSync);
        break;
      case ReadState::Sync3:
        changeReadState(ch, 0x44, ReadState::Sync4, ReadState::NoSync);
        break;
      case ReadState::Sync4:
      case ReadState::Value:
        _last_read_value = ch;
        _read_state = ReadState::EndSync;
        break;
      case ReadState::EndSync:
        changeReadState(ch, 0x55, ReadState::NoSync, ReadState::NoSync);
        break;
      default:
        break;
    }
  }
  return 0;
}
void SerialTestReceiver::start()
{
  pthread_attr_t receiveloop_attr;
  pthread_attr_init(&receiveloop_attr);

  struct sched_param param;
  (void)pthread_attr_getschedparam(&receiveloop_attr, &param);
  param.sched_priority = SCHED_PRIORITY_MAX - 80;
  (void)pthread_attr_setschedparam(&receiveloop_attr, &param);

  pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(2048));

  pthread_create(&_thread, &receiveloop_attr, SerialTestReceiver::start_trampoline, (void*)this);

  pthread_attr_destroy(&receiveloop_attr);

  _receiver_start = true;
}
void SerialTestReceiver::stop()
{
  _receiver_start = false;
  pthread_join(_thread, nullptr);
}
void* SerialTestReceiver::start_trampoline(void* context)
{
  SerialTestReceiver* self = reinterpret_cast<SerialTestReceiver*>(context);
  self->run();
  return nullptr;
}
void SerialTestReceiver::run()
{
  /* set thread name */
  {
    char thread_name[17];
    snprintf(thread_name, sizeof(thread_name), "ser_tes_receiver");
    px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
  }

  // poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
  const int timeout = 50;

  /* the serial port buffers internally as well, we just need to fit a small chunk */
  uint8_t buf[64];

  struct pollfd fds[1] = {};

  fds[0].fd = _serial_test.get_uart_fd();
  fds[0].events = POLLIN;

  ssize_t nread = 0;

  while (!_serial_test.should_exit())
  {
    int ret = poll(&fds[0], 1, timeout);

    if (ret > 0)
    {
      /* non-blocking read. read may return negative values */
      nread = ::read(fds[0].fd, buf, sizeof(buf));

      if (nread == -1 && errno == ENOTCONN)
      {
        usleep(100000);
      }

      if (nread > 0)
      {
        /* if read failed, this loop won't execute */
        handleReceive(buf, nread);
      }
    }
    else if (ret == -1)
    {
      usleep(10000);
    }
  }
}
void SerialTestReceiver::print_info()
{
  PX4_INFO("Receiving Serial Test\n");
  PX4_INFO("Now received value %d\n", _last_read_value);
  PX4_INFO("Read error count: %d", _read_error);
  PX4_INFO("Read success count: %d", _read_success);
}

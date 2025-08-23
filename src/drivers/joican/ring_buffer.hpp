#include <stdint.h>
#include <nuttx/irq.h>

// single producer in interupt context, single consumer in thread
template <typename T, int32_t Num>
class RingBuffer
{
public:
  void reset()
  {
    wridx_ = reidx_;
  }
  bool empty()
  {
    return wridx_ == reidx_;
  }
  void push(const T& element)
  {
    buf_[wridx_] = element;
    // increment wridx_
    wridx_++;
    if (wridx_ == Num)
    {
      wridx_ = 0;
      // overwrite, discard data
      if (wridx_ == reidx_)
      {
	reidx_++;
      }
      return;
    }
    else
    {
      // overwrite, discard data
      if (wridx_ == reidx_)
      {
	reidx_++;
      }
      if (reidx_ == Num)
      {
	reidx_ = 0;
      }
    }
    return;
  }
  bool pop(T& data)
  {
    irqstate_t flags = enter_critical_section();

    if (empty())
      return false;

    data = buf_[reidx_];

    // increment reidx
    reidx_++;

    if (reidx_ == Num)
    {
      reidx_ = 0;
    }
    leave_critical_section(flags);

    return true;
  }

private:
  T buf_[Num];
  volatile int32_t wridx_{0};
  volatile int32_t reidx_{0};
};

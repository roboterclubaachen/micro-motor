#pragma once
#include <cstdlib>

namespace sim
{

template<size_t id>
class Pin
{
public:
	static void
	set(bool on);
	static bool
	read();

private:
	static inline bool val{false};
};

template<size_t id>
void
Pin<id>::set(bool on)
{
	val = on;
}

template<size_t id>
bool
Pin<id>::read()
{
	return val;
}

}  // namespace sim

#pragma once
#include <cstdint>
#include <array>
template<typename T, int32_t Size>
class RollingAverage
{
private:
	std::array<T, Size> values_{};
	int32_t valueCount_ = 0;
	int32_t nextIndex_ = 0;

public:
	inline void
	addValue(T val)
	{
		if (valueCount_ < Size) valueCount_++;
		values_[nextIndex_] = val;
		nextIndex_ = (nextIndex_ + 1) % Size;
	}
	inline T
	average() const
	{
		T temp{};
		for (int32_t i = 0; i < valueCount_; i++) { temp += values_[i]; }
		return temp / valueCount_;
	}
};
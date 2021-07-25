#pragma once

/// Compute absolute electrical angle from 16 bit encoder value
class EncoderAngleEvaluation
{
public:
    /// \param ticksPerCycle Encoder ticks per electrical cycle of the motor
    constexpr EncoderAngleEvaluation(int32_t ticksPerCycle, uint16_t initialValue = 0)
        : cycleTicks_{ticksPerCycle}, lastValue_{initialValue}
    {
    }

    constexpr void update(uint16_t encoderValue)
    {
        const auto tickDifference = (int16_t)(uint16_t)(encoderValue - lastValue_);
        velocity_ = tickDifference;
        lastValue_ = encoderValue;

        angle_ += tickDifference;
        angle_ = angle_ % cycleTicks_;
    }

    constexpr void resetZeroAngle(uint16_t encoderValue)
    {
        lastValue_ = encoderValue;
        angle_ = 0;
    }

    /// Absolute electrical angle in ticks (-ticksPerCycle, ticksPerCycle)
    constexpr int32_t angle()
    {
        return angle_;
    }

    constexpr int16_t velocity()
    {
        return velocity_;
    }

private:
    int32_t position_ = 0;
    int32_t angle_ = 0;
    int32_t cycleTicks_;
    uint16_t lastValue_;
    int16_t velocity_ = 0;
};


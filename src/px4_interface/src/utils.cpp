#include <cmath>

template <typename T>
class LowPassFilter
{
   public:
    LowPassFilter(T samplingRate, T cutoffFrequency)
        : samplingRate(samplingRate), cutoffFrequency(cutoffFrequency)
    {
        calculateAlpha();
        prevOutput = static_cast<T>(0);
    }

    T filter(T input)
    {
        T output   = alpha * input + (static_cast<T>(1) - alpha) * prevOutput;
        prevOutput = output;
        return output;
    }

   private:
    T samplingRate;
    T cutoffFrequency;
    T alpha;
    T prevOutput;

    void calculateAlpha()
    {
        T rc = static_cast<T>(1) /
               (static_cast<T>(2) * static_cast<T>(M_PI) * cutoffFrequency);
        T dt  = static_cast<T>(1) / samplingRate;
        alpha = dt / (rc + dt);
    }
};

#pragma once
#include <stdint.h>
#include <math.h>

namespace angles
{
    static constexpr float HALF_RADIANS = M_PI, HALF_DEGREES = 180;

    enum class domain : uint8_t
    {
        continuous, // ex: 0 to 360
        mirror,     // ex: -180 to 180
    };

    enum class unit : uint8_t
    {
        radians,
        degrees,
    };

    /**
     * identity of a pair. useful for comparisons.
     */
    template <domain D, unit U>
    constexpr uint8_t identity()
    {
        return (static_cast<uint8_t>(D) << 1) | static_cast<uint8_t>(U);
    }

    template <unit U>
    constexpr float half()
    {
        if constexpr (U == unit::radians)
            return HALF_RADIANS;
        else
            return HALF_DEGREES;
    }

    template <domain D, unit U>
    struct angle
    {
        float value;

        constexpr static float min_a()
        {
            if constexpr (D == domain::continuous)
                return 0;
            else
                return -half<U>();
        }

        constexpr static float max_a()
        {
            if constexpr (D == domain::continuous)
                return 2 * half<U>();
            else
                return half<U>();
        }

        /**
         * convert an angle from one domain to the other
         */
        template <domain target_d>
        constexpr angle<target_d, U> convert() const
        {
            if constexpr (target_d == D)
                return {this->value};
            else if constexpr (target_d == domain::continuous)
                return {this->value < 0
                            ? angle<domain::continuous, U>::max_a() + this->value
                            : this->value};
            else
                return {this->value >= half<U>()
                            ? -angle<domain::continuous, U>::max_a() + this->value
                            : this->value};
        }

        /**
         * translate an angle from one unit to the other
         */
        template <unit target_u>
        constexpr angle<D, target_u> translate() const
        {
            if constexpr (target_u == U)
                return *this;
            else if constexpr (target_u == unit::radians)
                return {this->value * angle<D, unit::radians>::max_a() / angle<D, unit::degrees>::max_a()};
            else
                return {this->value * angle<D, unit::degrees>::max_a() / angle<D, unit::radians>::max_a()};
        }

        constexpr angle<domain::mirror, U> travel(const angle<D, U> &to) const
        {
            auto from_v = this->normalize().template convert<domain::continuous>().value;
            auto to_v = to.normalize().template convert<domain::continuous>().value;
            auto zeroed = to_v - from_v;
            if (zeroed > angle<domain::mirror, U>::max_a())
            {
                return {zeroed - angle<domain::continuous, U>::max_a()};
            }
            else if (zeroed < angle<domain::mirror, U>::min_a())
            {
                return {zeroed + angle<domain::continuous, U>::max_a()};
            }
            else
            {
                return {zeroed};
            }
        }

        constexpr angle<D, U> normalize() const
        {
            if constexpr (D == domain::continuous)
            {
                auto max = angle<domain::continuous, U>::max_a();
                auto numtimes = this->value/max;
                auto moded = this->value - (floor(numtimes) * max);
                auto value = moded;

                if (value < 0)
                    value += angle<domain::continuous, U>::max_a();
                return {value};
            }
            else
                return this->template convert<domain::continuous>().normalize().template convert<domain::mirror>();
        }

        /**
         * automatically normalizes
         */
        constexpr static angle<D, U> from(const float v)
        {
            angle<D, U> a{v};
            return a.normalize();
        }
    };

    /**
     * convert an angle from one domain to the other
     */
    template <domain target_d, domain src_d, unit U>
    constexpr angle<target_d, U> convert(const angle<src_d, U> &a)
    {
        return a.convert();
    }

    /**
     * translate an angle from one unit to the other
     */
    template <domain D, unit src_u, unit target_u>
    constexpr angle<D, target_u> translate(const angle<D, src_u> &a)
    {
        return a.translate();
    }

    template <domain D, unit U>
    constexpr angle<domain::mirror, U> travel(const angle<D, U> &from, const angle<D, U> &to)
    {
        return from.travel(to);
    }

    template <domain D, unit U>
    constexpr angle<D, U> normalize(angle<D, U> &a)
    {
        return a.normalize();
    }

    /**
     * using [EMA](https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average)
     */
    class AngleMovingAvg
    {
        float _alpha, _running_avg_x, _running_avg_y;
        bool _init;

    public:
        static constexpr domain DOMAIN = domain::mirror;
        static constexpr unit UNIT = unit::radians;

        AngleMovingAvg(float alpha)
            : _alpha(alpha), _running_avg_x(0), _running_avg_y(0), _init(false) {}

        void add(angle<DOMAIN, UNIT> a)
        {
            auto x = cos(a.value);
            auto y = sin(a.value);
            if (!this->_init)
            {
                this->_running_avg_x = x;
                this->_running_avg_y = y;
                this->_init = true;
            }
            else
            {
                this->_running_avg_x = this->_alpha * x + (1.0 - this->_alpha) * this->_running_avg_x;
                this->_running_avg_y = this->_alpha * y + (1.0 - this->_alpha) * this->_running_avg_y;
            }
        }

        angle<DOMAIN, UNIT> calc()
        {
            if (!this->_init)
                return {NAN};
            return angle<DOMAIN, UNIT>::from(atan2(this->_running_avg_y, this->_running_avg_x));
        }

        bool is_init()
        {
            return this->_init;
        }
    };
};

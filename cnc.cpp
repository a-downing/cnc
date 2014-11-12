#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <functional>

#include <unistd.h>
#include <fcntl.h>

const double pi = atan(1) * 4;

struct quaternion_t;

struct vector_t {
    double x = 0;
    double y = 0;
    double z = 0;

    static double getVectorsRadius(const vector_t &_v1, const vector_t &_v2, double length) {
        vector_t v1 = _v1.normalized();
        vector_t v2 = _v2.normalized();
        double dot = v1.dot(v2);
        return tan(acos(dot) / 2) * length;
    }

    constexpr vector_t() = default;

    constexpr vector_t(double x, double y, double z) : x(x), y(y), z(z) {

    }

    bool operator==(const vector_t &v) const {
        if(x == v.x && y == v.y && z == v.z) {
            return true;
        } else {
            return false;
        }
    }

    bool operator!=(const vector_t &v) const {
        return !(*this == v);
    }

    vector_t operator-() const {
        return {-x, -y, -z};
    }

    vector_t operator-(const vector_t &v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    vector_t operator+(const vector_t &v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    vector_t operator*(double s) const {
        return {x*s, y*s, z*s};
    }

    double dot(const vector_t &v2) const {
        return x*v2.x + y*v2.y + z*v2.z;
    }

    double length() const {
        return ::sqrt(x*x + y*y + z*z);
    }

    double lengthSquared() const {
        return x*x + y*y + z*z;
    }

    vector_t &normalize() {
        double len = length();

        if(len == 0) {
            return *this;
        }

        x /= len;
        y /= len;
        z /= len;

        return *this;
    }

    vector_t normalized() const {
        return vector_t(*this).normalize();
    }

    vector_t sqrt() {
        double len = ::sqrt(length());
        return vector_t(x / len, y / len, z / len);
    }

    vector_t &round() {
        x = floor(x + 0.5);
        y = floor(y + 0.5);
        z = floor(z + 0.5);

        return *this;
    }

    vector_t rounded() const {
        vector_t v = {
            floor(x + 0.5),
            floor(y + 0.5),
            floor(z + 0.5)
        };

        return v;
    }

    void print() const {
        printf("%.9f %.9f %.9f\n", x, y, z);
    }

    const vector_t& rotate(const quaternion_t&);
};

struct quaternion_t
{
    double x;
    double y;
    double z;
    double w;

    constexpr quaternion_t() : x(0), y(0), z(0), w(1) {}

    constexpr quaternion_t(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}

    const quaternion_t& operator=(const quaternion_t& q2)
    {
        x = q2.x;
        y = q2.y;
        z = q2.z;
        w = q2.w;

        return *this;
    }

    constexpr quaternion_t conjugate() const
    {
        return quaternion_t(-x, -y, -z, w);
    }

    quaternion_t(const vector_t& axis, double angle)
    {
        angle *= 0.5;
        double s = sin(angle);

        x = axis.x * s;
        y = axis.y * s;
        z = axis.z * s;
        w = cos(angle);
    }

    quaternion_t operator*(const quaternion_t& q2) const
    {
        quaternion_t res;

        res.x = w * q2.x + x * q2.w + y * q2.z - z * q2.y;
        res.y = w * q2.y + y * q2.w + z * q2.x - x * q2.z;
        res.z = w * q2.z + z * q2.w + x * q2.y - y * q2.x;
        res.w = w * q2.w - x * q2.x - y * q2.y - z * q2.z;

        return res;
    }
};

const vector_t& vector_t::rotate(const quaternion_t& q)
{
    quaternion_t vec(x, y, z, 0.0);
    quaternion_t result;

    result = vec * q.conjugate();
    result = q * result;

    x = result.x;
    y = result.y;
    z = result.z;

    return *this;
}

template<typename T>
class ProximityPeakDetector {
    vector_t target;
    vector_t pos;
    bool peak = false;
    T *target_obj = nullptr;
    double tolerance;

public:
    ProximityPeakDetector(double tol): tolerance(tol) {

    }

    void setTarget(T &t, const vector_t &v1, const vector_t &v2) {
        target = v1;
        target_obj = &t;
        pos = v2;
        peak = false;
    }

    void reset() {
        target = {0, 0, 0};
        pos = {0, 0, 0};
        target_obj = nullptr;
        peak = false;
    }

    T &getTargetObject() const {
        return *target_obj;
    }

    const vector_t &getTargetPosition() const {
        return target;
    }

    bool hasTarget() const {
        return (target_obj == nullptr) ? false : true;
    }

    void update(const vector_t &v) {
        const double d_curr = (target - pos).length();
        const double d_new = (target - v).length();

        if(!hasTarget()) {
            return;
        }

        if(d_new <= d_curr && d_new > tolerance) {
            pos = v;
        } else {
            peak = true;
        }
    }

    bool getPeak() {
        return peak;
    }

    const vector_t &getPosition() const {
        return pos;
    }
};

struct ToolPath {
    class Segment {
    public:
        enum class Type {
            LINE,
            ARC
        };

    private:
        Type m_type;
        vector_t m_v1;
        vector_t m_v2;
        vector_t m_v3;
        vector_t m_v4;
        vector_t m_v5;
        double m_angle;
        double m_speed;
        double m_acceleration;
        double m_entry_speed;
        double m_exit_speed;

    public:
        Segment(
                const vector_t &from,
                const vector_t &to,
                double speed,
                double acceleration
        ):
            m_type(Type::LINE),
            m_v1(from),
            m_v5(to),
            m_speed(speed),
            m_acceleration(acceleration) {
        }

        Segment(const vector_t &from,
                const vector_t &center,
                const vector_t &axis,
                const vector_t &translate,
                const vector_t &to,
                double angle,
                double speed,
                double acceleration
        ):
            m_type(Type::ARC),
            m_v1(from),
            m_v2(center),
            m_v3(axis),
            m_v4(translate),
            m_v5(to),
            m_angle(angle),
            m_speed(speed),
            m_acceleration(acceleration) {
        }

        const vector_t &getArcCenter() const {
            return m_v2;
        }

        double getArcRadius() const {
            return m_v2.length();
        }

        double getArcRadians() const {
            return m_angle;
        }

        double getSpeed() const {
            return m_speed;
        }

        double getAcceleration() const {
            return m_acceleration;
        }

        void setEntrySpeed(double s) {
            m_entry_speed = s;
        }

        void setExitSpeed(double s) {
            m_exit_speed = s;
        }

        double getEntrySpeed() const {
            return m_entry_speed;
        }

        double getExitSpeed() const {
            return m_exit_speed;
        }

        Type getType() {
            return m_type;
        }

        vector_t getEntryVector() const {
            vector_t result;

            if(m_type == Type::LINE) {
                const vector_t &start = m_v1;
                const vector_t &end = m_v5;
                result = (end - start).normalized();
            } else if(m_type == Type::ARC) {
                const vector_t &start = m_v1;
                const vector_t &center = m_v2;
                const vector_t &axis = m_v3;
                const vector_t &translate = m_v4;
                const double angle = 1.0 / center.length();


                vector_t v = -center;
                v.rotate({axis, angle});
                v = v + (center + start);
                v = v + (translate * (angle / m_angle));

                result =  (v - start).normalized();
            }

            return result;
        }

        vector_t getExitVector() const {
            vector_t result;

            if(m_type == Type::LINE) {
                result = getEntryVector();
            } else if(m_type == Type::ARC) {
                const vector_t &start = m_v1;
                const vector_t &center = m_v2;
                const vector_t &axis = m_v3;
                const vector_t &translate = m_v4;
                const vector_t &to = m_v5;
                const double angle = m_angle - (1.0 / center.length());


                vector_t v = -center;
                v.rotate({axis, angle});
                v = v + (center + start);
                v = v + (translate * (angle / m_angle));

                result = (to - v).normalized();
            }

            return result;
        }

        const vector_t &getEntryPosition() const {
            return m_v1;
        }

        const vector_t &getExitPosition() const {
            return m_v5;
        }

        double getDistance() const {
            double distance;

            if(m_type == Type::LINE) {
                const vector_t &start = m_v1;
                const vector_t &end = m_v5;
                distance = (end - start).length();
            } else if(m_type == Type::ARC) {
                const vector_t &center = m_v2;
                distance = m_angle * center.length();
            }

            return distance;
        }

        double getMaxMidpointSpeed() const {
            double distance_half = getDistance() / 2;
            double speed_exit = getExitSpeed();
            double speed_max = sqrt(speed_exit*speed_exit - 2 * -m_acceleration * distance_half);
            return std::min(speed_max, m_speed);
        }

        double getDistanceAccelToSpeed(double speed) {
            double speed_enter = getEntrySpeed();
            double distance = (speed*speed - speed_enter*speed_enter) / (2 * m_acceleration);
            return distance;
        }

        double getDistanceDeccelFromSpeed(double speed) {
            double speed_exit = getExitSpeed();
            double distance = (speed_exit*speed_exit - speed*speed) / (2 * -m_acceleration);
            return distance;
        }

        double getSpeedAccelDistance(double speed, double distance) {
            return sqrt(speed*speed + 2 * m_acceleration * distance);
        }

        double getSpeedDeccelDistance(double speed, double distance) {
            return sqrt(speed*speed + 2 * -m_acceleration * distance);
        }

        double getSpeedAtDistance(double d) {
            double speed_max = getMaxMidpointSpeed();
            double distance = getDistance();
            double distance_accel = getDistanceAccelToSpeed(speed_max);
            double distance_decel = getDistanceDeccelFromSpeed(speed_max);
            double distance_coast = distance - (distance_accel + distance_decel);

            if(d <= distance_accel) {
                return getSpeedAccelDistance(getEntrySpeed(), d);
            } else if(d > distance_accel && d <= distance_accel + distance_coast) {
                return speed_max;
            } else {
                return getSpeedDeccelDistance(speed_max, d - (distance_accel + distance_coast));
            }
        }

        double getTimeAccelDistance(double speed, double distance) {
            return (sqrt(2*m_acceleration*distance + speed*speed) - speed) / m_acceleration;
        }

        double getTimeDeccelDistance(double speed, double distance) {
            return (sqrt(2*-m_acceleration*distance + speed*speed) - speed) / -m_acceleration;
        }

        double getTimeToDistance(double d) {
            double speed_max = getMaxMidpointSpeed();
            double distance = getDistance();
            double distance_accel = getDistanceAccelToSpeed(speed_max);
            double distance_decel = getDistanceDeccelFromSpeed(speed_max);
            double distance_coast = distance - (distance_accel + distance_decel);

            if(d <= distance_accel) {
                return getTimeAccelDistance(getEntrySpeed(), d);
            } else if(d > distance_accel && d <= distance_accel + distance_coast) {
                return getTimeAccelDistance(getEntrySpeed(), distance_accel) + (d - distance_accel) / speed_max;
            } else {
                return getTimeDeccelDistance(speed_max, distance - d) + getTimeAccelDistance(getEntrySpeed(), distance_accel) + (d - distance_accel) / speed_max;
            }
        }

        vector_t interpolate(double distance) const {
            vector_t result;

            if(m_type == Type::LINE) {
                const vector_t &start = m_v1;
                result = start + (getEntryVector() * distance);
            } else if(m_type == Type::ARC) {
                const vector_t &start = m_v1;
                const vector_t &center = m_v2;
                const vector_t &axis = m_v3;
                const vector_t &translate = m_v4;
                const double angle = distance / center.length();

                result = -center;
                result.rotate({axis, angle});
                result = result + (center + start);
                result = result + (translate * (angle / m_angle));
            }

            return result;
        }
    };

    enum class Units {
        MM,
        INCH,
        STEP
    };

    struct step_plan_t {
        vector_t pos;
        vector_t pos_exact;
        vector_t pos_misc;
        double speed;
        double t;

        step_plan_t(const vector_t &v1, const vector_t &v2, double s): pos(v1), pos_exact(v2), speed(s) {
        }
    };

    struct step_t {
        vector_t pos;
        double t;

        step_t() = default;
        step_t(const vector_t &v, double t): pos(v), t(t) {
        }
    };

    double steps_per_mm;
    double scaler;
    vector_t pos;
    double vel;
    double acc;
    std::vector<Segment> segments;
    std::vector<step_plan_t> steps;

    ToolPath() {
        init();
    }

    void init() {
        pos = {0, 0, 0};
        vel = 0;
        acc = 0;
    }

    void zero(int x, int y, int z) {
        pos.x = x;
        pos.y = y;
        pos.z = z;
    }

    vector_t getPosition() const {
        return pos * (1 / scaler);
    }

    void setStepsPerMM(double s) {
        steps_per_mm = s;
    }

    void setUnits(Units units) {
        switch(units) {
            case Units::STEP:
                scaler = 1;
                break;
            case Units::MM:
                scaler = steps_per_mm;
            case Units::INCH:
                scaler = steps_per_mm * 25.4;
        }
    }

    void setVelocity(double v) {
        vel = v * scaler;
    }

    void setAcceleration(double a) {
        acc = a * scaler;
    }

    void relArc(const vector_t &_center, const vector_t &_axis, double angle, const vector_t &_translate = {0,0,0}) {
        vector_t center = _center * scaler;
        vector_t translate = _translate * scaler;
        vector_t axis = _axis.normalized();
        vector_t v = -center;

        v.rotate({axis, angle});
        v = v + (center + pos);
        v = v + translate;

        segments.emplace_back(
            pos,
            center,
            axis,
            translate,
            v,
            angle,
            vel,
            acc
        );

        pos = v;
    }

    void absArc(const vector_t &center, const vector_t &axis, double angle, const vector_t &translate = {0,0,0}) {
        relArc(center - getPosition(), axis, angle, translate - getPosition());
    }

    void absArc(const vector_t &center, const vector_t &axis, double angle) {
        relArc(center - getPosition(), axis, angle);
    }

    void relMove(const vector_t &_to) {
        vector_t to = _to * scaler;
        to = to + pos;
        segments.emplace_back(pos, to, vel, acc);
        pos = to;
    }

    void absMove(const vector_t &to) {
        relMove(to - getPosition());
    }

    void calculateJunctions() {
        // the first and last positions of the toothpath are at zero velocity
        segments.front().setEntrySpeed(0);
        segments.back().setExitSpeed(0);

        // get the vector pairs for each junction between segments
        // treat each corner between junctions as an arc that deviates
        // at most 1 step from the real corner in order to find the max
        // velocity at the corner given the max accerlation of both segments
        for(int i = 0; i < segments.size() - 1; i++) {
            Segment &prev = segments[i];
            Segment &next = segments[i + 1];

            // get the radius of a circle tangent to the ends of the two vectors
            // that form the corner. entry should be negated so that both vectors
            // point away from the corner. using a length of 1 so that corners are
            // rounded off by no more than the length of one step.
            double radius = vector_t::getVectorsRadius(-prev.getExitVector(), next.getEntryVector(), 1);

            // corner acceleration must be <= to both segments max acceleration
            double max_acceleration = std::min(prev.getAcceleration(), next.getAcceleration());

            // with the given radius calculate the maximum speed
            // around the corner given a max centripetal acceleration
            double max_speed = sqrt(max_acceleration * radius);
            max_speed = std::min({max_speed, prev.getSpeed(), next.getSpeed()});

            prev.setExitSpeed(max_speed);
            next.setEntrySpeed(max_speed);
        }
    }

    void planPath(std::function<void(const step_t &)> callback) {
        calculateJunctions();

        double t_total = 0;
        step_plan_t last_inserted({0, 0, 0}, {0, 0, 0}, 0);
        for(int idx = 0; idx < segments.size(); idx++) {
            Segment &segment = segments[idx];
            const double dd = 0.01;
            double distance = segment.getDistance();
            std::vector<step_plan_t> segment_steps;

            if(idx == 0) {
                vector_t pos = segments.front().getEntryPosition();
                segment_steps.push_back(step_plan_t(pos, pos, 0));
                last_inserted = segment_steps.back();
            }

            // discretize steps
            int iterations = (segment.getDistance() / dd) + 1;
            for(int i = 0; i < iterations + 1; i++) {
                double d = (distance / iterations) * i;
                vector_t v = segment.interpolate(d);
                vector_t step_pos = v.rounded();

                if(step_pos != last_inserted.pos) {
                    double speed = segment.getSpeedAtDistance(d);
                    segment_steps.emplace_back(step_pos, vector_t(0, 0, 0), 0);
                    last_inserted = segment_steps.back();

                    if(segment.getType() == Segment::Type::LINE) {
                        vector_t v1 = segment.getEntryPosition() - step_pos;
                        vector_t v2 = segment.getExitPosition() - segment.getEntryPosition();
                        double t = -v1.dot(v2) / v2.lengthSquared();
                        vector_t closest = segment.interpolate(segment.getDistance() * t);
                        segment_steps.back().pos_exact = closest;
                    } else if(segment.getType() == Segment::Type::ARC) {
                        vector_t v1 = step_pos - (segment.getArcCenter() + segment.getEntryPosition());
                        v1 = v1 * (segment.getArcRadius() / v1.length());
                        v1 = v1 + segment.getEntryPosition() + segment.getArcCenter();
                        segment_steps.back().pos_exact = v1;
                    }

                    segment_steps.back().speed = speed;
                }
            }

            std::vector<step_plan_t> steps_tmp;
            steps_tmp.reserve(segment_steps.size());

            // optimize the step path
            for(int i = 0; i < segment_steps.size();) {
                if(i > steps.size() - 3) {
                     steps_tmp.emplace_back(segment_steps[i]);
                     i++;
                     continue;
                }

                const vector_t v = segment_steps[i + 2].pos - segment_steps[i].pos;
                const double d1 = (segment_steps[i + 1].pos - segment_steps[i + 1].pos_exact).length();
                const double d2 = (segment_steps[i + 2].pos - segment_steps[i + 2].pos_exact).length();

                steps_tmp.emplace_back(segment_steps[i]);

                if(std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)}) > 1) {
                    i++;
                } else if(d2 <= d1) {
                    i += 2;
                } else {
                    i++;
                }
            }

            segment_steps = std::move(steps_tmp);

            for(size_t i = 0; i < segment_steps.size(); i++) {
                if(i == 0) {
                    continue;
                }

                step_plan_t &next_step = segment_steps[i];
                step_plan_t &prev_step = segment_steps[i - 1];
                const double d = (next_step.pos - prev_step.pos).length();
                const double d_exact = (next_step.pos_exact - prev_step.pos_exact).length();
                const double v_avg = prev_step.speed + ((next_step.speed - prev_step.speed) / 2);

                //prev_step.t = d_exact / v_avg;
                prev_step.t = d / v_avg;

                callback(step_t(prev_step.pos, prev_step.t));

                if(idx == segments.size()) {
                    callback(step_t(next_step.pos, 0));
                }
            }
        }
    }
};

template<typename T>
constexpr T get_bits(T n, int first, int last) {
    return (n >> last) & ((1 << (first - last + 1)) - 1);
}

template<typename T>
constexpr T set_bits(T n, int first, int last, T to) {
    return (n & ~(((1 << (first - last + 1)) - 1) << last)) | ((to & ((1 << (first - last + 1)) - 1)) << last);
}

template<typename T>
constexpr T set_bits(int first, int last, T to) {
    return (to & ((1 << (first - last + 1)) - 1)) << last;
}

struct INST_SIG {
    static const uint32_t IDLE = 0;
    static const uint32_t STEP = IDLE + 1;
    static const uint32_t DIR  = IDLE + 2;
    static const uint32_t SYNC = IDLE + 3;
};

class MachineController {
    std::string m_serial_port;
    int m_fd = -1;
    std::vector<uint8_t> m_instructions;
    uint8_t m_dir = set_bits(7, 4, INST_SIG::DIR);
    vector_t m_last_step_dir = {0, 0, 0};
    uint32_t m_timer_freq = 0;

public:
    MachineController(const std::string &port): m_serial_port(port) {

    }

    bool connect() {
        m_fd = open(m_serial_port.c_str(), O_RDWR | O_SYNC | O_NOCTTY);

        if(m_fd == -1) {
            return false;
        } else {
            return true;
        }
    }

    bool sendProgram() {
        size_t written = 0;

        while(written < m_instructions.size()) {
            size_t left = m_instructions.size() - written;
            size_t len = write(m_fd, &m_instructions[written], left);

            if(len == -1) {
                return false;
            }

            written += len;
        }

        return true;
    }

    bool connected() const {
        return (m_fd == -1) ? false : true;
    }

    void reset() {
        if(connected()) {
            close(m_fd);
        }

        m_fd = -1;
        m_instructions.clear();
        m_last_step_dir = {0, 0, 0};
    }

    void setTimerFreq(uint32_t f) {
        m_timer_freq = f;
    }

    void cycleStart() {
        uint8_t instr = set_bits(7, 4, INST_SIG::SYNC);
        m_instructions.push_back(instr);
    }

    void addStep(const vector_t &step, double t) {
        uint8_t dir = m_dir;
        dir = set_bits(dir, 0, 0, (step.x == 1) ? (uint8_t)1 : (uint8_t)0);
        dir = set_bits(dir, 1, 1, (step.y == 1) ? (uint8_t)1 : (uint8_t)0);
        dir = set_bits(dir, 2, 2, (step.z == 1) ? (uint8_t)1 : (uint8_t)0);

        if(dir != m_dir) {
            m_instructions.push_back(dir);
            m_dir = dir;
        }

        // step is a 32-bit instruction [31:28 0b0001][27:24 ?zyx][23:0 timer ticks delay]
        uint8_t byte_0 = set_bits(7, 4, INST_SIG::STEP);
        byte_0 = set_bits(byte_0, 0, 0, (step.x != 0) ? (uint8_t)1 : (uint8_t)0);
        byte_0 = set_bits(byte_0, 1, 1, (step.y != 0) ? (uint8_t)1 : (uint8_t)0);
        byte_0 = set_bits(byte_0, 2, 2, (step.z != 0) ? (uint8_t)1 : (uint8_t)0);

        uint32_t ticks = m_timer_freq * t;

        uint8_t byte_1 = get_bits(ticks, 23, 16);
        uint8_t byte_2 = get_bits(ticks, 15, 8);
        uint8_t byte_3 = get_bits(ticks, 7, 0);

        m_instructions.push_back(byte_0);
        m_instructions.push_back(byte_1);
        m_instructions.push_back(byte_2);
        m_instructions.push_back(byte_3);
    }
};

int main() {
    ToolPath path;
    MachineController controller("/dev/ttyUSB0");

    controller.setTimerFreq(64000000);

    path.setStepsPerMM(320);
    path.setUnits(ToolPath::Units::INCH);
    path.setVelocity(75.0 / 60);
    path.setAcceleration(1);

    auto helicalDrill = [&](double tool_dia, double hole_dia, double depth, double doc) {
        double arc_radius = (hole_dia - tool_dia) / 2;
        double arc_radians = (depth / doc) * 2*pi;

        vector_t center = path.getPosition();
        path.relMove({arc_radius, 0, 0});
        path.relArc(center - path.getPosition(), {0, 0, 1}, arc_radians, {0, 0, -depth});
        path.relArc(center - path.getPosition(), {0, 0, 1}, 2*pi);
        path.absMove(center);
    };

    /*path.absMove({1, 0, 0});
    path.absMove({0, -1, 0});
    path.absMove({-1, 0, 0});
    path.absMove({0, 1, 0});*/

    path.absMove({-1.5, 1.5, 0});
    helicalDrill(0.25, 1, 0.125, 0.02);

    path.absMove({1.5, 1.5, 0});
    helicalDrill(0.25, 1, 0.125, 0.02);

    path.absMove({1.5, -1.5, 0});
    helicalDrill(0.25, 1, 0.125, 0.02);

    path.absMove({-1.5, -1.5, 0});
    helicalDrill(0.25, 1, 0.125, 0.02);

    path.absMove({0, 0, 0});

    //path.relMove({4, 0, 0});

    ToolPath::step_t prev_step;
    int step_no = 0;

    printf("planning path...\n");
    path.planPath([&](const ToolPath::step_t &step) {
        if(step_no == 0) {
            prev_step = step;
            step_no++;
            return;
        }

        const ToolPath::step_t &next_step = step;
        vector_t v = next_step.pos - prev_step.pos;

        controller.addStep(v, prev_step.t);

        prev_step = step;
        step_no++;
    });

    controller.cycleStart();

    printf("sending program...\n");
    if(!controller.connect()) {
        printf("Failed to connect\n");
        return 1;
    }

    if(!controller.sendProgram()) {
        printf("Failed to send program\n");
        return 1;
    }

    printf("success\n");

    return 0;
}

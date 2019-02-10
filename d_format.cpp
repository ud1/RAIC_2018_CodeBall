#include <cassert>
#include "d_format.hpp"
#include <sstream>
#include <boost/endian/conversion.hpp>

inline uint64_t reinterpretDoubleAsUint64_t(double v)
{
    union { double d; uint64_t u; } u;
    u.d = v;
    return u.u;
}

inline double reinterpretUint64_tAsDouble(uint64_t v)
{
    union { double d; uint64_t u; } u;
    u.u = v;
    return u.d;
}

void writeString(std::ostream &ostream, const std::string &str)
{
    ostream.put('s');
    uint32_t size = boost::endian::native_to_big((uint32_t) str.size());
    ostream.write(reinterpret_cast<const char *>(&size), sizeof(size));
    ostream.write(str.c_str(), str.size());
}

void writeDouble(std::ostream &ostream, double val)
{
    ostream.put('d');
    uint64_t v = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(val));
    ostream.write(reinterpret_cast<const char *>(&v), sizeof(val));
}

void writeInt(std::ostream &ostream, uint32_t val)
{
    ostream.put('i');
    val = boost::endian::native_to_big(val);
    ostream.write(reinterpret_cast<const char *>(&val), sizeof(val));
}

void writeP(std::ostream &ostream, const P &p)
{
    ostream.put('p');
    uint64_t vx = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(p.x));
    uint64_t vy = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(p.z));
    ostream.write(reinterpret_cast<const char *>(&vx), sizeof(vx));
    ostream.write(reinterpret_cast<const char *>(&vy), sizeof(vy));
}

void writeP3(std::ostream &ostream, const P3 &p)
{
    ostream.put('v');
    uint64_t vx = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(p.x));
    uint64_t vy = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(p.y));
    uint64_t vz = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(p.z));
    ostream.write(reinterpret_cast<const char *>(&vx), sizeof(vx));
    ostream.write(reinterpret_cast<const char *>(&vy), sizeof(vy));
    ostream.write(reinterpret_cast<const char *>(&vz), sizeof(vz));
}

void writeM4(std::ostream &ostream, const M4 &m)
{
    ostream.put('t');
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            uint64_t v = boost::endian::native_to_big(reinterpretDoubleAsUint64_t(m[i][j]));
            ostream.write(reinterpret_cast<const char *>(&v), sizeof(v));
        }
    }
}

void writeVarT(std::ostream &ostream, const var_t &v)
{
    if (std::holds_alternative<std::string>(v))
    {
        writeString(ostream, std::get<std::string>(v));
    }
    else if (std::holds_alternative<double>(v))
    {
        writeDouble(ostream, std::get<double>(v));
    }
    else if (std::holds_alternative<uint32_t>(v))
    {
        writeInt(ostream, std::get<uint32_t>(v));
    }
    else if (std::holds_alternative<P>(v))
    {
        writeP(ostream, std::get<P>(v));
    }
    else if (std::holds_alternative<P3>(v))
    {
        writeP3(ostream, std::get<P3>(v));
    }
    else if (std::holds_alternative<M4>(v))
    {
        writeM4(ostream, std::get<M4>(v));
    }
    else
    {
        assert(false);
    }
}

std::string toString(const var_t &v)
{
    std::ostringstream oss;
    if (std::holds_alternative<std::string>(v))
    {
        return std::get<std::string>(v);
    }
    else if (std::holds_alternative<double>(v))
    {
        oss << std::get<double>(v);
    }
    else if (std::holds_alternative<uint32_t>(v))
    {
        uint32_t val = std::get<uint32_t>(v);
        oss << val << "(#" << std::hex << std::get<uint32_t>(v) << std::dec << ")";
    }
    else if (std::holds_alternative<P>(v))
    {
        P p = std::get<P>(v);
        oss << "P(" << p.x << ", " << p.z << ")";
    }
    else if (std::holds_alternative<P3>(v))
    {
        P3 p = std::get<P3>(v);
        oss << "V(" << p.x << ", " << p.y << ", " << p.z << ")";
    }
    else if (std::holds_alternative<M4>(v))
    {
        M4 m = std::get<M4>(v);
        oss << "M("
            << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << m[0][3]
            << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << m[1][3]
            << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << m[2][3]
            << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << m[3][3]
            << ")";
    }
    else
    {
        assert(false);
    }

    return oss.str();
}

var_t readVarT(std::istream &istream)
{
    int c = istream.get();
    if (c == std::char_traits<char>::eof())
    {
        return std::monostate();
    }

    switch (static_cast<char>(c))
    {
        case 's':
        {
            uint32_t size;
            istream.read(reinterpret_cast<char *>(&size), sizeof(size));
            size = boost::endian::big_to_native(size);

            if (size > 100000)
            {
                LOG("Too long string " << size);
                istream.ignore(size);
                return std::string("Too long string");
            }

            std::string str(size, ' ');
            istream.read(&str[0], size);

            return str;
        }
        case 'd':
        {
            uint64_t v;
            istream.read(reinterpret_cast<char *>(&v), sizeof(v));
            double val = reinterpretUint64_tAsDouble(boost::endian::big_to_native(v));
            return val;
        }
        case 'i':
        {
            uint32_t val;
            istream.read(reinterpret_cast<char *>(&val), sizeof(val));
            val = boost::endian::big_to_native(val);
            return val;
        }
        case 'p':
        {
            P val;
            uint64_t vx, vy;
            istream.read(reinterpret_cast<char *>(&vx), sizeof(vx));
            istream.read(reinterpret_cast<char *>(&vy), sizeof(vy));
            val.x = reinterpretUint64_tAsDouble(boost::endian::big_to_native(vx));
            val.z = reinterpretUint64_tAsDouble(boost::endian::big_to_native(vy));
            return val;
        }
        case 'v':
        {
            P3 val;
            uint64_t vx, vy, vz;
            istream.read(reinterpret_cast<char *>(&vx), sizeof(vx));
            istream.read(reinterpret_cast<char *>(&vy), sizeof(vy));
            istream.read(reinterpret_cast<char *>(&vz), sizeof(vz));
            val.x = reinterpretUint64_tAsDouble(boost::endian::big_to_native(vx));
            val.y = reinterpretUint64_tAsDouble(boost::endian::big_to_native(vy));
            val.z = reinterpretUint64_tAsDouble(boost::endian::big_to_native(vz));
            return val;
        }
        case 't':
        {
            M4 m;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    uint64_t v;
                    istream.read(reinterpret_cast<char *>(&v), sizeof(v));
                    m[i][j] = reinterpretUint64_tAsDouble(boost::endian::big_to_native(v));
                }
            }
            return m;
        }
        default: {
            assert(false);
        }
    }

    return std::monostate();
}

void writeSObj(std::ostream &ostream, const SObj &obj)
{
    ostream.put('m');
    writeInt(ostream, obj.size());
    for (auto &&p : obj)
    {
        writeString(ostream, p.first);
        writeVarT(ostream, p.second);
    }
}

SObj readSObj(std::istream &istream)
{
    SObj result;

    int c = istream.get();
    if (c == std::char_traits<char>::eof())
    {
        return result;
    }

    assert (static_cast<char>(c) == 'm');

    uint32_t len = std::get<uint32_t>(readVarT(istream));

    for (size_t i = 0; i < len; ++i)
    {
        std::string key = std::get<std::string>(readVarT(istream));
        var_t val = readVarT(istream);
        result.insert(std::make_pair(key, val));
    }

    return result;
}

void writeObj(std::ostream &ostream, const Obj &obj)
{
    ostream.put('o');
    writeString(ostream, obj.type);
    writeSObj(ostream, obj.props);
    writeInt(ostream, obj.subObjs.size());
    for (auto &&p : obj.subObjs)
    {
        writeString(ostream, p.first);
        writeSObj(ostream, p.second);
    }
}

Obj readObj(std::istream &istream)
{
    Obj result;

    int c = istream.get();
    if (c == std::char_traits<char>::eof())
    {
        return result;
    }

    assert (static_cast<char>(c) == 'o');

    result.type = std::get<std::string>(readVarT(istream));
    result.props = readSObj(istream);
    uint32_t len = std::get<uint32_t>(readVarT(istream));

    for (size_t i = 0; i < len; ++i)
    {
        std::string key = std::get<std::string>(readVarT(istream));
        SObj val = readSObj(istream);
        result.subObjs.insert(std::make_pair(key, val));
    }

    return result;
}

std::ostream &operator <<(std::ostream &str, const SObj &obj) {
    str << "{";
    int i = 0;
    for (const auto &p : obj)
    {
        if (i > 0)
            str <<", ";
        ++i;

        str << p.first << ": " << toString(p.second);
    }

    str << "}";
    return str;
}

std::ostream &operator <<(std::ostream &str, const Obj &obj) {
    str << "Obj(" << obj.type;

    if (!obj.props.empty())
    {
        str << ", props: " << obj.props;
    }

    if (!obj.subObjs.empty())
    {
        str << ", objs: {";
        int i = 0;
        for (const auto &p : obj.subObjs)
        {
            if (i > 0)
                str <<", ";
            ++i;

            str << p.first << ": " << p.second;
        }
        str << "}";
    }

    str << ")";
    return str;
}

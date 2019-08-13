#include <memory>
#include <mutex>

#include <eigen3/Eigen/Core>

typedef int64_t time_t;

template <typename data_t, size_t dimension, size_t length>
class CircularBufferEigenTimestamped
{
public:
    CircularBufferEigenTimestamped(void) {};
    ~CircularBufferEigenTimestamped(void) {};

    using data_element_t = Eigen::Matrix<data_t, dimension, 1>;

    inline void Insert(time_t timestamp, const data_element_t &data)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        // add data
        _timestamps(_head) = timestamp;
        _data.col(_head) = data;

        // update head and tail
        _head = (_head + 1) % length;
        if (_head == _tail)
        {
            _tail = (_tail + 1) % length;
        }
    }

    std::pair<data_element_t, bool> GetOldestElement(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(data_element_t(), false);
        }

        return std::make_pair(_data.col(_tail), true);
    }

    std::pair<data_element_t, bool> GetNewestElement(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(data_element_t(), false);
        }

        return std::make_pair(_data.col((_head - 1 + length) % length), true);

    }

    std::pair<time_t, bool> GetNewestTimestamp(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(0, false);
        }

        return std::make_pair(_timestamps((_head - 1 + length) % length), true);
    }

    std::pair<time_t, bool> GetOldestTimestamp(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(0, false);
        }

        return std::make_pair(_timestamps(_tail), true);
    }

    bool Empty(void)
    {
        return _head == _tail;
    };
    bool Full(void)
    {
        return ((_head + 1) % _size) == _tail;
    }
    void Reset()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _head = _tail;
    }

private:
    std::mutex _mutex;
    size_t _head = 0;
    size_t _tail = 0;
    size_t _size;

    Eigen::Matrix<time_t, length, 1> _timestamps;
    Eigen::Matrix<data_t, dimension, length> _data;
};

template <class T, size_t length>
class CircularBufferClassTimestamped
{
public:
    CircularBufferClassTimestamped(void):
        _data(std::unique_ptr<T []>(new T[length])) {};
    ~CircularBufferClassTimestamped(void) {};

    inline void insert(time_t timestamp, const T &data)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        // add data
        _timestamps(_head) = timestamp;
        _data[_head] = data;

        // update head and tail
        _head = (_head + 1) % length;
        if (_head == _tail)
        {
            _tail = (_tail + 1) % length;
        }
    }

    std::pair<T, bool> GetOldestElement(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(T(), false);
        }

        return std::make_pair(_data[_tail], true);
    }

    std::pair<T, bool> GetNewestElement(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        
        if (empty())
        {
            return std::make_pair(T(), false);
        }

        return std::make_pair(_data[(_head - 1 + length) % length], true);

    }

    std::pair<time_t, bool> GetNewestTimestamp(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(0, false);
        }

        return std::make_pair(_timestamps((_head - 1 + length) % length), true);
    }

    std::pair<time_t, bool> GetOldestTimestamp(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if (empty())
        {
            return std::make_pair(0, false);
        }

        return std::make_pair(_timestamps(_tail), true);
    }

    bool Empty(void)
    {
        return _head == _tail;
    };
    bool Full(void)
    {
        return ((_head + 1) % _size) == _tail;
    }
    void Reset()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _head = _tail;
    }

private:
    std::mutex _mutex;
    size_t _head = 0;
    size_t _tail = 0;
    size_t _size;

    Eigen::Matrix<time_t, length, 1> _timestamps;
    std::unique_ptr<T []> _data;
};
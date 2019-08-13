#include <memory>
#include <mutex>

template <class T, size_t length>
class CircularBuffer
{
public:
    CircularBuffer(void):
        _data(std::unique_ptr<T []>(new T[length])) {};
    ~CircularBuffer(void) {};

    inline void Insert(const T &data)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        // add data
        _data[_head] = data;

        // update head and tail
        _head = (_head + 1) % length;
        if (_head == _tail)
        {
            _tail = (_tail + 1) % length;
        }
    }

    inline void Insert_array(const T data[], size_t size)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        for (size_t i = 0; i < size; i++)
        {
            // add data
            _data[_head] = *(data++);

            // update head and tail
            _head = (_head + 1) % length;
            if (_head == _tail)
            {
                _tail = (_tail + 1) % length;
            }
        }
    }

    inline T& operator[] (size_t i)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        return _data[(_tail + i) % length];
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

    bool Empty(void)
    {
        return _head == _tail;
    }

    bool Full(void)
    {
        return ((_head + 1) % _size) == _tail;
    }

    size_t Size(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return (_head + length - _tail) % length;
    }

    void Reset(void)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _head = _tail;
    }

private:
    std::mutex _mutex;
    size_t _head = 0;
    size_t _tail = 0;
    size_t _size;

    std::unique_ptr<T []> _data;
};
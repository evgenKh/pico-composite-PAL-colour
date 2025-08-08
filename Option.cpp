#include "Option.h"
#include <cstdio>


    void Option::Int32Formatter(char* outBuf, const size_t outBufSize, const T value)
    {
        snprintf(outBuf, outBufSize, "%d", value);
    }

    void Option::Int32MHzFormatter(char* outBuf, const size_t outBufSize, const T value)
    {
        snprintf(outBuf, outBufSize, "%dMHz", value);
    }

    void Option::FormatToCstr(char* outBuf, const size_t outBufSize) const
    {
        if(m_formatter)
        {
            m_formatter(outBuf, outBufSize, m_currentValue);
        }
    }
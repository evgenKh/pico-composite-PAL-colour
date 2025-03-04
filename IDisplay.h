#ifndef IDisplay_H
#define IDisplay_H


class IDisplay{
    public:
    virtual uint16_t GetWidth() const = 0;
    virtual uint16_t GetHeight() const = 0;
    virtual void SetFontHeight(uint8_t fontHeight) = 0;
    virtual void DrawText(uint16_t x, uint16_t y, const char* string) = 0;
    virtual void SelectRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h) = 0;
    virtual void Flush() = 0;
    virtual void Clear() = 0;
};

#endif
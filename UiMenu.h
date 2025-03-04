#ifndef UiMenu_H
#define UiMenu_H

#include <pico/types.h>
#include <vector>

class IDisplay;
class UiMenuItem;
class OptionsRegistry;

struct UiMenuPage
{
    const char* m_title;
    std::vector<UiMenuItem> m_items;
    uint8_t m_selectedItem = 0;
};

class UiMenu
{
    public:
    IDisplay* m_display = nullptr;
    std::vector<UiMenuPage> m_pages;
    int8_t m_currentPage = -1;
    bool m_isDirty = true;
    OptionsRegistry* m_optionsRegistry = nullptr;

    void Update();
    void Draw();
    void OpenPage(int8_t pageNum);
    void SelectNextItem();
    void SelectPrevItem();
    void ItemIncrement();
    void ItemDecrement();
    UiMenuPage* GetPage(int8_t pageNum);
    UiMenuItem* GetItem(int8_t pageNum, uint8_t itemNum);
};


#endif
#include "UiMenu.h"
#include "UiMenuItem.h"
#include "IDisplay.h"
#include "Option.h"
#include "string.h"
#include "stdio.h"
#include "OptionsRegistry.h"

UiMenuPage *UiMenu::GetPage(int8_t pageNum)
{
    if(pageNum < m_pages.size()) 
    {
        return &m_pages[pageNum];
    }
    return nullptr;
}

UiMenuItem *UiMenu::GetItem(int8_t pageNum, uint8_t itemNum)
{
    UiMenuPage* page = GetPage(pageNum);
    if(page) 
    {
        if(itemNum < page->m_items.size())
        {
            return &page->m_items[itemNum];
        }
    }
    return nullptr;
}

void UiMenu::Update()
{
    if(m_isDirty)
    {
        UiMenuPage* page = GetPage(m_currentPage);
        if(page) 
        {

        }
    }
}

void UiMenu::Draw()
{
    if(!m_isDirty) return;
    UiMenuPage* page = GetPage(m_currentPage);
    if(!page) return;
    if(page->m_selectedItem >= page->m_items.size()) return;
    if(!m_display) return;

    const uint16_t fontHeight = 8;
    const uint16_t paddingX = 1;
    const uint16_t paddingY = 1;
    const uint16_t itemHeight = fontHeight + paddingY*2;
    const uint16_t itemsOnScreen = 64/itemHeight;
    const uint16_t itemWidth = 128;
    const uint16_t fontWidth = 6;
    //const uint16_t textMaxLength = 3;

    const uint8_t startItem = 0;//page->m_selectedItem;

    m_display->Clear();
    uint16_t currentY = (fontHeight + 1*paddingY - 1);
    for(uint8_t i = startItem; i < startItem+itemsOnScreen; i++)
    {
        const UiMenuItem* item = GetItem(m_currentPage, i);
        char itemTextFormatted[64]="";
        if(item)
        {
            const Option* option = (m_optionsRegistry ? m_optionsRegistry->GetOption(item->m_option) : nullptr);
            //char itemTextFormatted[64] = "test";
            char valueFormatted[64] = "";
            if(option)
            {
                sprintf(valueFormatted, "=%d", option->m_currentValue);
            }

            const char* titleStr = "";
            if(item->m_title && strlen(item->m_title))
            {
                titleStr = item->m_title;
            }
            else if(option && option->m_name && strlen(option->m_name))
            {
                titleStr = option->m_name;
            }
            
            sprintf(itemTextFormatted, "%s%s", titleStr, valueFormatted);

            m_display->DrawText(paddingX, currentY+paddingY-1, itemTextFormatted);

            if(i == page->m_selectedItem)
            {
                m_display->SelectRect(0, currentY - fontHeight - paddingY, itemWidth, (fontHeight + 2*paddingY));
            }

        }
        currentY += (fontHeight + 2*paddingY);
    }
    m_display->Flush();
    m_isDirty = false;
}

void UiMenu::OpenPage(int8_t pageNum)
{
    if(GetPage(pageNum))
    {
        m_currentPage = pageNum;
        m_isDirty = true;
    }
    else
    {
        m_currentPage = -1;
        m_isDirty = true;
    }
}

void UiMenu::SelectNextItem()
{    
    UiMenuPage* page = GetPage(m_currentPage);
    if(page) 
    {
        page->m_selectedItem++;
        if(!GetItem(m_currentPage, page->m_selectedItem))
        {
            page->m_selectedItem = 0;
        }
        m_isDirty = true;
    }
}

void UiMenu::ItemIncrement()
{
    UiMenuPage* page = GetPage(m_currentPage);
    if(page) 
    {
        UiMenuItem* item = GetItem(m_currentPage, page->m_selectedItem);
        if(item)
        {
            Option* option = (m_optionsRegistry ? m_optionsRegistry->GetOption(item->m_option) : nullptr);
            if(item->m_pageNumToOpen >= 0)
            {
                OpenPage(item->m_pageNumToOpen);
                m_isDirty = true;
            }
            else if(option)
            {
                if(option->m_min == option->m_max)
                {
                    return;
                }
                if(option->m_currentValue < option->m_max)
                {
                    option->m_currentValue += option->m_incrementStep;
                }
                else
                {
                    option->m_currentValue = option->m_min;
                }
                option->m_valueToSave = option->m_currentValue;  
                option->m_isSaveRequested = true;              
                m_isDirty = true;
            }
            else if(item->m_callbackFunc)
            {
                item->m_callbackFunc(item->m_callbackFuncUserData);
            }
        }
    }
}

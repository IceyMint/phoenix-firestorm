/** 
 * @file llkeybind.h
 * @brief Information about key combinations.
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License only.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
 * $/LicenseInfo$
 */

#ifndef LL_KEYBIND_H
#define LL_KEYBIND_H

#include "indra_constants.h"

// KeyData - single key combination (mouse/mask/keyboard)
class LL_COMMON_API LLKeyData
{
public:
    LLKeyData();
    LLKeyData(EMouseClickType mouse, KEY key, MASK mask);
    LLKeyData(const LLSD &key_data);

    LLSD asLLSD() const;
    bool isEmpty() const;
    bool empty() const { return isEmpty(); };
    void reset();
    LLKeyData& operator=(const LLKeyData& rhs);
    bool operator==(const LLKeyData& rhs);
    bool operator!=(const LLKeyData& rhs);

    EMouseClickType mMouse;
    KEY mKey;
    MASK mMask;
};

// One function can bind to multiple Key options
class LLKeyBind
{
public:
    LLKeyBind() {}
    LLKeyBind(const LLSD &key_bind);

    bool operator==(const LLKeyBind& rhs);
    bool operator!=(const LLKeyBind& rhs);
    bool isEmpty() const;
    bool empty() const { return isEmpty(); };

    LLSD asLLSD() const;

    bool canHandle(EMouseClickType mouse, KEY key, MASK mask) const;
    bool canHandleKey(KEY key, MASK mask) const;
    bool canHandleMouse(EMouseClickType mouse, MASK mask) const;

    // these methods enshure there will be no repeats
    bool addKeyData(EMouseClickType mouse, KEY key, MASK mask);
    bool addKeyData(const LLKeyData& data);
    void replaceKeyData(EMouseClickType mouse, KEY key, MASK mask, U32 index);
    void replaceKeyData(const LLKeyData& data, U32 index);
    bool hasKeyData(U32 index) const;
    void clear() { mData.clear(); };
    LLKeyData getKeyData(U32 index) const;
    U32 getDataCount();

private:
    typedef std::vector<LLKeyData> data_vector_t;
    data_vector_t mData;
};


#endif // LL_KEYBIND_H

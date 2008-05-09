/** 
 * @file lluictrlfactory.h
 * @brief Factory class for creating UI controls
 *
 * $LicenseInfo:firstyear=2003&license=viewergpl$
 * 
 * Copyright (c) 2003-2007, Linden Research, Inc.
 * 
 * Second Life Viewer Source Code
 * The source code in this file ("Source Code") is provided by Linden Lab
 * to you under the terms of the GNU General Public License, version 2.0
 * ("GPL"), unless you have obtained a separate licensing agreement
 * ("Other License"), formally executed by you and Linden Lab.  Terms of
 * the GPL can be found in doc/GPL-license.txt in this distribution, or
 * online at http://secondlife.com/developers/opensource/gplv2
 * 
 * There are special exceptions to the terms and conditions of the GPL as
 * it is applied to this Source Code. View the full text of the exception
 * in the file doc/FLOSS-exception.txt in this software distribution, or
 * online at http://secondlife.com/developers/opensource/flossexception
 * 
 * By copying, modifying or distributing this software, you acknowledge
 * that you have read and understood your obligations described above,
 * and agree to abide by those obligations.
 * 
 * ALL LINDEN LAB SOURCE CODE IS PROVIDED "AS IS." LINDEN LAB MAKES NO
 * WARRANTIES, EXPRESS, IMPLIED OR OTHERWISE, REGARDING ITS ACCURACY,
 * COMPLETENESS OR PERFORMANCE.
 * $/LicenseInfo$
 */

#ifndef LLUICTRLFACTORY_H
#define LLUICTRLFACTORY_H

#include <iosfwd>
#include <stack>

#include "llcallbackmap.h"
#include "llfloater.h"

class LLView;
class LLPanel;

class LLUICtrlFactory : public LLSingleton<LLUICtrlFactory>
{
public:
	LLUICtrlFactory();
	// do not call!  needs to be public so run-time can clean up the singleton
	virtual ~LLUICtrlFactory();

	void setupPaths();

	void buildFloater(LLFloater* floaterp, const LLString &filename, 
					const LLCallbackMap::map_t* factory_map = NULL, BOOL open = TRUE);
	BOOL buildPanel(LLPanel* panelp, const LLString &filename,
					const LLCallbackMap::map_t* factory_map = NULL);

	void removePanel(LLPanel* panelp) { mBuiltPanels.erase(panelp->getHandle()); }
	void removeFloater(LLFloater* floaterp) { mBuiltFloaters.erase(floaterp->getHandle()); }

	class LLMenuGL *buildMenu(const LLString &filename, LLView* parentp);
	class LLPieMenu *buildPieMenu(const LLString &filename, LLView* parentp);

	// Does what you want for LLFloaters and LLPanels
	// Returns 0 on success
	S32 saveToXML(LLView* viewp, const LLString& filename);

	// Rebuilds all currently built panels.
	void rebuild();

	static BOOL getAttributeColor(LLXMLNodePtr node, const LLString& name, LLColor4& color);

	LLPanel* createFactoryPanel(LLString name);

	virtual LLView* createCtrlWidget(LLPanel *parent, LLXMLNodePtr node);
	virtual LLView* createWidget(LLPanel *parent, LLXMLNodePtr node);

	static bool getLayeredXMLNode(const LLString &filename, LLXMLNodePtr& root);

private:

	typedef std::map<LLHandle<LLPanel>, LLString> built_panel_t;
	built_panel_t mBuiltPanels;

	typedef std::map<LLHandle<LLFloater>, LLString> built_floater_t;
	built_floater_t mBuiltFloaters;

	std::deque<const LLCallbackMap::map_t*> mFactoryStack;

	static std::vector<LLString> mXUIPaths;

	LLPanel* mDummyPanel;
};


#endif //LLUICTRLFACTORY_H

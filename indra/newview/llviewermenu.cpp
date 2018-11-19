/** 
 * @file llviewermenu.cpp
 * @brief Builds menus out of items.
 *
 * $LicenseInfo:firstyear=2002&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2014, Linden Research, Inc.
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

#include "llviewerprecompiledheaders.h"

#ifdef INCLUDE_VLD
#define VLD_FORCE_ENABLE 1
#include "vld.h"
#endif

#include "llviewermenu.h" 

// linden library includes
#include "llavatarnamecache.h"	// IDEVO
#include "llfloaterreg.h"
#include "llfloatersidepanelcontainer.h"
#include "llcombobox.h"
#include "llinventorypanel.h"
#include "llnotifications.h"
#include "llnotificationsutil.h"
#include "llviewereventrecorder.h"

// newview includes
#include "llagent.h"
#include "llagentaccess.h"
#include "llagentcamera.h"
#include "llagentui.h"
#include "llagentwearables.h"
#include "llagentpilot.h"
// [SL:KB] - Patch: Appearance-PhantomAttach | Checked: Catznip-5.0
#include "llattachmentsmgr.h"
// [/SL:KB]
#include "llcompilequeue.h"
#include "llconsole.h"
#include "lldaycyclemanager.h"
#include "lldebugview.h"
#include "llenvmanager.h"
#include "llfacebookconnect.h"
#include "llfilepicker.h"
#include "llfirstuse.h"
#include "llfloaterabout.h"
#include "llfloaterbuy.h"
#include "llfloaterbuycontents.h"
#include "llbuycurrencyhtml.h"
#include "llfloatergodtools.h"
#include "llfloaterimcontainer.h"
#include "llfloaterland.h"
#include "llfloaterimnearbychat.h"
#include "llfloaterpathfindingcharacters.h"
#include "llfloaterpathfindinglinksets.h"
#include "llfloaterpay.h"
#include "llfloaterreporter.h"
#include "llfloatersearch.h"
#include "llfloaterscriptdebug.h"
#include "llfloatersnapshot.h"
#include "llfloatertools.h"
#include "llfloaterworldmap.h"
#include "llfloaterbuildoptions.h"
#include "llavataractions.h"
#include "lllandmarkactions.h"
#include "llgroupmgr.h"
#include "lltooltip.h"
#include "lltoolface.h"
#include "llhints.h"
#include "llhudeffecttrail.h"
#include "llhudmanager.h"
#include "llimview.h"
#include "llinventorybridge.h"
#include "llinventorydefines.h"
#include "llinventoryfunctions.h"
#include "llpanellogin.h"
#include "llpanelblockedlist.h"
#include "llpanelmaininventory.h"
#include "llmarketplacefunctions.h"
#include "llmenuoptionpathfindingrebakenavmesh.h"
#include "llmoveview.h"
#include "llparcel.h"
#include "llrootview.h"
#include "llsceneview.h"
#include "llscenemonitor.h"
#include "llselectmgr.h"
#include "llspellcheckmenuhandler.h"
#include "llstatusbar.h"
#include "lltextureview.h"
#include "lltoolbarview.h"
#include "lltoolcomp.h"
#include "lltoolmgr.h"
#include "lltoolpie.h"
#include "lltoolselectland.h"
#include "lltrans.h"
#include "llviewerdisplay.h" //for gWindowResized
#include "llviewergenericmessage.h"
#include "llviewerhelp.h"
#include "llviewermenufile.h"	// init_menu_file()
#include "llviewermessage.h"
#include "llviewernetwork.h"
#include "llviewerobjectlist.h"
#include "llviewerparcelmgr.h"
#include "llviewerstats.h"
#include "llvoavatarself.h"
#include "llvoicevivox.h"
#include "llworldmap.h"
#include "pipeline.h"
#include "llviewerjoystick.h"
#include "llwaterparammanager.h"
#include "llwlanimator.h"
#include "llwlparammanager.h"
#include "llfloatercamera.h"
#include "lluilistener.h"
#include "llappearancemgr.h"
#include "lltrans.h"
#include "lleconomy.h"
#include "lltoolgrab.h"
#include "llwindow.h"
#include "llpathfindingmanager.h"
#include "llstartup.h"
#include "boost/unordered_map.hpp"
#include <boost/regex.hpp>
#include "llcleanup.h"
// [RLVa:KB] - Checked: 2011-05-22 (RLVa-1.3.1a)
#include "fsavatarrenderpersistence.h"
#include "rlvactions.h"
#include "rlvhandler.h"
#include "rlvlocks.h"
// [/RLVa:KB]

// Firestorm includes
#include "fsassetblacklist.h"
#include "fsdata.h"
#include "fslslbridge.h"
#include "fscommon.h"
#include "fsfloaterexport.h"
#include "fsfloatercontacts.h"
#include "fsfloaterplacedetails.h"
#include "fspose.h"
#include "lfsimfeaturehandler.h"
#include "llavatarpropertiesprocessor.h"
#include "llcheckboxctrl.h"
#include "llfloatergridstatus.h"
#include "llfloaterpreference.h"
#include "lllogininstance.h"
#include "llscenemonitor.h"
#include "llsdserialize.h"
#include "lltexturecache.h"
#include "llvovolume.h"
#include "particleeditor.h"

using namespace LLAvatarAppearanceDefines;

typedef LLPointer<LLViewerObject> LLViewerObjectPtr;

static boost::unordered_map<std::string, LLStringExplicit> sDefaultItemLabels;

BOOL enable_land_build(void*);
BOOL enable_object_build(void*);

LLVOAvatar* find_avatar_from_object( LLViewerObject* object );
LLVOAvatar* find_avatar_from_object( const LLUUID& object_id );

void handle_test_load_url(void*);

//
// Evil hackish imported globals

//extern BOOL	gHideSelectedObjects;
//extern BOOL gAllowSelectAvatar;
//extern BOOL gDebugAvatarRotation;
extern BOOL gDebugClicks;
extern BOOL gDebugWindowProc;
extern BOOL gShaderProfileFrame;

//extern BOOL gDebugTextEditorTips;
//extern BOOL gDebugSelectMgr;

//
// Globals
//

LLMenuBarGL		*gMenuBarView = NULL;
LLViewerMenuHolderGL	*gMenuHolder = NULL;
LLMenuGL		*gPopupMenuView = NULL;
LLMenuGL		*gEditMenu = NULL;
LLMenuBarGL		*gLoginMenuBarView = NULL;

// Context menus
LLContextMenu	*gMenuAvatarSelf	= NULL;
LLContextMenu	*gMenuAvatarOther = NULL;
LLContextMenu	*gMenuObject = NULL;
LLContextMenu	*gMenuAttachmentSelf = NULL;
LLContextMenu	*gMenuAttachmentOther = NULL;
LLContextMenu	*gMenuLand	= NULL;
LLContextMenu	*gMenuMuteParticle = NULL;

// <FS:Zi> Pie menu
// Pie menus
PieMenu		*gPieMenuAvatarSelf	= NULL;
PieMenu		*gPieMenuAvatarOther = NULL;
PieMenu		*gPieMenuObject = NULL;
PieMenu		*gPieMenuAttachmentSelf = NULL;
PieMenu		*gPieMenuAttachmentOther = NULL;
PieMenu		*gPieMenuLand	= NULL;
PieMenu		*gPieMenuMuteParticle = NULL;
// <FS:Zi> Pie menu

const std::string SAVE_INTO_TASK_INVENTORY("Save Object Back to Object Contents");

LLMenuGL* gAttachSubMenu = NULL;
LLMenuGL* gDetachSubMenu = NULL;
LLMenuGL* gTakeOffClothes = NULL;
LLContextMenu* gAttachScreenPieMenu = NULL;
LLContextMenu* gAttachPieMenu = NULL;
LLContextMenu* gAttachBodyPartPieMenus[9];
LLContextMenu* gDetachPieMenu = NULL;
LLContextMenu* gDetachScreenPieMenu = NULL;
LLContextMenu* gDetachBodyPartPieMenus[9];

// <FS:Zi> Pie menu
PieMenu* gPieAttachScreenMenu = NULL;
PieMenu* gPieAttachMenu = NULL;
PieMenu* gPieAttachBodyPartMenus[PIE_MAX_SLICES];
PieMenu* gPieDetachMenu = NULL;
PieMenu* gPieDetachScreenMenu = NULL;
PieMenu* gPieDetachBodyPartMenus[PIE_MAX_SLICES];
// <FS:Zi> Pie menu

LLMenuItemCallGL* gAutorespondMenu = NULL;
LLMenuItemCallGL* gAutorespondNonFriendsMenu = NULL;
//
// Local prototypes

// File Menu
void handle_compress_image(void*);


// Edit menu
void handle_dump_group_info(void *);
void handle_dump_capabilities_info(void *);

// Advanced->Consoles menu
void handle_region_dump_settings(void*);
void handle_region_dump_temp_asset_data(void*);
void handle_region_clear_temp_asset_data(void*);

// Object pie menu
BOOL sitting_on_selection();

void near_sit_object();
//void label_sit_or_stand(std::string& label, void*);
// buy and take alias into the same UI positions, so these
// declarations handle this mess.
BOOL is_selection_buy_not_take();
S32 selection_price();
BOOL enable_take();
void handle_object_show_inspector();
void handle_avatar_show_inspector();
bool confirm_take(const LLSD& notification, const LLSD& response, LLObjectSelectionHandle selection_handle);

void handle_buy_object(LLSaleInfo sale_info);
void handle_buy_contents(LLSaleInfo sale_info);

// Land pie menu
void near_sit_down_point(BOOL success, void *);

// Avatar pie menu

// Debug menu


void velocity_interpolate( void* );
void handle_visual_leak_detector_toggle(void*);
void handle_rebake_textures(void*);
BOOL check_admin_override(void*);
void handle_admin_override_toggle(void*);
#ifdef TOGGLE_HACKED_GODLIKE_VIEWER
void handle_toggle_hacked_godmode(void*);
BOOL check_toggle_hacked_godmode(void*);
bool enable_toggle_hacked_godmode(void*);
#endif

void toggle_show_xui_names(void *);
BOOL check_show_xui_names(void *);

// Debug UI

void handle_buy_currency_test(void*);
void handle_save_to_xml(void*);
void handle_load_from_xml(void*);

void handle_god_mode(void*);

// God menu
void handle_leave_god_mode(void*);


void handle_reset_view();

void handle_duplicate_in_place(void*);


void handle_object_owner_self(void*);
void handle_object_owner_permissive(void*);
void handle_object_lock(void*);
void handle_object_asset_ids(void*);
void force_take_copy(void*);
#ifdef _CORY_TESTING
void force_export_copy(void*);
void force_import_geometry(void*);
#endif

void handle_force_parcel_owner_to_me(void*);
void handle_force_parcel_to_content(void*);
void handle_claim_public_land(void*);

void handle_god_request_avatar_geometry(void *);	// Hack for easy testing of new avatar geometry
void reload_vertex_shader(void *);
void handle_disconnect_viewer(void *);

void force_error_breakpoint(void *);
void force_error_llerror(void *);
void force_error_bad_memory_access(void *);
void force_error_infinite_loop(void *);
void force_error_software_exception(void *);
void force_error_driver_crash(void *);

void handle_force_delete(void*);
void print_object_info(void*);
void print_agent_nvpairs(void*);
void toggle_debug_menus(void*);
void upload_done_callback(const LLUUID& uuid, void* user_data, S32 result, LLExtStat ext_status);
void dump_select_mgr(void*);

void dump_inventory(void*);
void toggle_visibility(void*);
BOOL get_visibility(void*);

// Avatar Pie menu
void request_friendship(const LLUUID& agent_id);

// Tools menu
void handle_selected_texture_info(void*);
void handle_selected_material_info();

void handle_dump_followcam(void*);
void handle_viewer_enable_message_log(void*);
void handle_viewer_disable_message_log(void*);

BOOL enable_buy_land(void*);

// Help menu

void handle_test_male(void *);
void handle_test_female(void *);
void handle_dump_attachments(void *);
void handle_dump_avatar_local_textures(void*);
void handle_debug_avatar_textures(void*);
void handle_grab_baked_texture(void*);
BOOL enable_grab_baked_texture(void*);
void handle_dump_region_object_cache(void*);

BOOL enable_save_into_task_inventory(void*);

BOOL enable_detach(const LLSD& = LLSD());
void menu_toggle_attached_lights(void* user_data);
void menu_toggle_attached_particles(void* user_data);

void avatar_tex_refresh(LLVOAvatar* avatar);	// <FS:CR> FIRE-11800

class LLMenuParcelObserver : public LLParcelObserver
{
public:
	LLMenuParcelObserver();
	~LLMenuParcelObserver();
	virtual void changed();
};

static LLMenuParcelObserver* gMenuParcelObserver = NULL;

static LLUIListener sUIListener;

LLMenuParcelObserver::LLMenuParcelObserver()
{
	LLViewerParcelMgr::getInstance()->addObserver(this);
}

LLMenuParcelObserver::~LLMenuParcelObserver()
{
	LLViewerParcelMgr::getInstance()->removeObserver(this);
}

void LLMenuParcelObserver::changed()
{
	LLParcel *parcel = LLViewerParcelMgr::getInstance()->getParcelSelection()->getParcel();
	// <FS:Ansariel> FIRE-4454: Cache controls because of performance reasons
	//gMenuHolder->childSetEnabled("Land Buy Pass", LLPanelLandGeneral::enableBuyPass(NULL) && !(parcel->getOwnerID()== gAgent.getID()));
	//
	//BOOL buyable = enable_buy_land(NULL);
	//gMenuHolder->childSetEnabled("Land Buy", buyable);
	//gMenuHolder->childSetEnabled("Buy Land...", buyable);

	static LLView* land_buy_pass = gMenuHolder->getChildView("Land Buy Pass");
	static LLView* land_buy_pass_pie = gMenuHolder->getChildView("Land Buy Pass Pie");
	static LLView* land_buy = gMenuHolder->getChildView("Land Buy");
	static LLView* land_buy_pie = gMenuHolder->getChildView("Land Buy Pie");

	BOOL pass_buyable = LLPanelLandGeneral::enableBuyPass(NULL) && parcel->getOwnerID() != gAgentID;
	land_buy_pass->setEnabled(pass_buyable);
	land_buy_pass_pie->setEnabled(pass_buyable);

	BOOL buyable = enable_buy_land(NULL);
	land_buy->setEnabled(buyable);
	land_buy_pie->setEnabled(buyable);
	// </FS:Ansariel> FIRE-4454: Cache controls because of performance reasons
}


void initialize_menus();

//-----------------------------------------------------------------------------
// Initialize main menus
//
// HOW TO NAME MENUS:
//
// First Letter Of Each Word Is Capitalized, Even At Or And
//
// Items that lead to dialog boxes end in "..."
//
// Break up groups of more than 6 items with separators
//-----------------------------------------------------------------------------

void set_underclothes_menu_options()
{
	if (gMenuHolder && gAgent.isTeen())
	{
		gMenuHolder->getChild<LLView>("Self Underpants")->setVisible(FALSE);
		gMenuHolder->getChild<LLView>("Self Undershirt")->setVisible(FALSE);
	}
	if (gMenuBarView && gAgent.isTeen())
	{
		gMenuBarView->getChild<LLView>("Menu Underpants")->setVisible(FALSE);
		gMenuBarView->getChild<LLView>("Menu Undershirt")->setVisible(FALSE);
	}
}

void set_merchant_SLM_menu()
{
    // All other cases (new merchant, not merchant, migrated merchant): show the new Marketplace Listings menu and enable the tool
    gMenuHolder->getChild<LLView>("MarketplaceListings")->setVisible(TRUE);
    LLCommand* command = LLCommandManager::instance().getCommand("marketplacelistings");
	gToolBarView->enableCommand(command->id(), true);
}

void check_merchant_status(bool force)
{
	// <FS:Ansariel> Don't show merchant outbox or SL Marketplace stuff outside SL
	if (!LLGridManager::getInstance()->isInSecondLife())
	{
		gMenuHolder->getChild<LLView>("MarketplaceListings")->setVisible(FALSE);
		return;
	}
	// </FS:Ansariel>

    if (!gSavedSettings.getBOOL("InventoryOutboxDisplayBoth"))
    {
        if (force)
        {
            // Reset the SLM status: we actually want to check again, that's the point of calling check_merchant_status()
            LLMarketplaceData::instance().setSLMStatus(MarketplaceStatusCodes::MARKET_PLACE_NOT_INITIALIZED);
        }
        // Hide SLM related menu item
        gMenuHolder->getChild<LLView>("MarketplaceListings")->setVisible(FALSE);
        
        // Also disable the toolbar button for Marketplace Listings
        LLCommand* command = LLCommandManager::instance().getCommand("marketplacelistings");
		gToolBarView->enableCommand(command->id(), false);
        
        // Launch an SLM test connection to get the merchant status
        LLMarketplaceData::instance().initializeSLM(boost::bind(&set_merchant_SLM_menu));
    }
}

void init_menus()
{
	// Initialize actions
	initialize_menus();

	///
	/// Popup menu
	///
	/// The popup menu is now populated by the show_context_menu()
	/// method.
	
	LLMenuGL::Params menu_params;
	menu_params.name = "Popup";
	menu_params.visible = false;
	gPopupMenuView = LLUICtrlFactory::create<LLMenuGL>(menu_params);
	gMenuHolder->addChild( gPopupMenuView );

	///
	/// Context menus
	///

	const widget_registry_t& registry =
		LLViewerMenuHolderGL::child_registry_t::instance();
	gEditMenu = LLUICtrlFactory::createFromFile<LLMenuGL>("menu_edit.xml", gMenuHolder, registry);
	gMenuAvatarSelf = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_avatar_self.xml", gMenuHolder, registry);
	gMenuAvatarOther = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_avatar_other.xml", gMenuHolder, registry);

	gDetachScreenPieMenu = gMenuHolder->getChild<LLContextMenu>("Object Detach HUD", true);
	gDetachPieMenu = gMenuHolder->getChild<LLContextMenu>("Object Detach", true);

	gMenuObject = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_object.xml", gMenuHolder, registry);

	gAttachScreenPieMenu = gMenuHolder->getChild<LLContextMenu>("Object Attach HUD");
	gAttachPieMenu = gMenuHolder->getChild<LLContextMenu>("Object Attach");

	gMenuAttachmentSelf = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_attachment_self.xml", gMenuHolder, registry);
	gMenuAttachmentOther = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_attachment_other.xml", gMenuHolder, registry);

	gMenuLand = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_land.xml", gMenuHolder, registry);

	gMenuMuteParticle = LLUICtrlFactory::createFromFile<LLContextMenu>(
		"menu_mute_particle.xml", gMenuHolder, registry);

// <FS:Zi> Pie menu
	gPieMenuAvatarSelf = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_avatar_self.xml", gMenuHolder, registry);
	gPieMenuAvatarOther = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_avatar_other.xml", gMenuHolder, registry);

	// added "Pie" to the control names to keep them unique
	gPieDetachScreenMenu = gMenuHolder->getChild<PieMenu>("Pie Object Detach HUD", true);
	gPieDetachMenu = gMenuHolder->getChild<PieMenu>("Pie Object Detach", true);

	gPieMenuObject = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_object.xml", gMenuHolder, registry);

	// added "Pie" to the control names to keep them unique
	gPieAttachScreenMenu = gMenuHolder->getChild<PieMenu>("Pie Object Attach HUD");
	gPieAttachMenu = gMenuHolder->getChild<PieMenu>("Pie Object Attach");

	gPieMenuAttachmentSelf = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_attachment_self.xml", gMenuHolder, registry);
	gPieMenuAttachmentOther = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_attachment_other.xml", gMenuHolder, registry);

	gPieMenuLand = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_land.xml", gMenuHolder, registry);

	gPieMenuMuteParticle = LLUICtrlFactory::createFromFile<PieMenu>(
		"menu_pie_mute_particle.xml", gMenuHolder, registry);
// </FS:Zi> Pie menu

	///
	/// set up the colors
	///
	LLColor4 color;

	// do not set colors in code, let the skin decide. -Zi
	/*
	LLColor4 context_menu_color = LLUIColorTable::instance().getColor("MenuPopupBgColor");
	
	gMenuAvatarSelf->setBackgroundColor( context_menu_color );
	gMenuAvatarOther->setBackgroundColor( context_menu_color );
	gMenuObject->setBackgroundColor( context_menu_color );
	gMenuAttachmentSelf->setBackgroundColor( context_menu_color );
	gMenuAttachmentOther->setBackgroundColor( context_menu_color );

	gMenuLand->setBackgroundColor( context_menu_color );

	color = LLUIColorTable::instance().getColor( "MenuPopupBgColor" );
	gPopupMenuView->setBackgroundColor( color );
	*/

	// <FS> Changed for grid manager
	// If we are not in production, use a different color to make it apparent.
	//if (LLGridManager::getInstance()->isInProductionGrid())
	//{
	//	color = LLUIColorTable::instance().getColor( "MenuBarBgColor" );
	//}
	//else
	//{
	//	color = LLUIColorTable::instance().getColor( "MenuNonProductionBgColor" );
	//}

	//LLView* menu_bar_holder = gViewerWindow->getRootView()->getChildView("menu_bar_holder");

	//gMenuBarView = LLUICtrlFactory::getInstance()->createFromFile<LLMenuBarGL>("menu_viewer.xml", gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());
	//gMenuBarView->setRect(LLRect(0, menu_bar_holder->getRect().mTop, 0, menu_bar_holder->getRect().mTop - MENU_BAR_HEIGHT));
	//gMenuBarView->setBackgroundColor( color );

	gMenuBarView = LLUICtrlFactory::getInstance()->createFromFile<LLMenuBarGL>("menu_viewer.xml", gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());
	// ONLY change the color IF we are in beta. Otherwise leave it alone so it can use the skinned color. -Zi
	if(LLGridManager::getInstance()->isInSLBeta())
	{
		color = LLUIColorTable::instance().getColor( "MenuNonProductionBgColor" );
		gMenuBarView->setBackgroundColor( color );
	}

	LLView* menu_bar_holder = gViewerWindow->getRootView()->getChildView("menu_bar_holder");
	gMenuBarView->setRect(LLRect(0, menu_bar_holder->getRect().mTop, 0, menu_bar_holder->getRect().mTop - MENU_BAR_HEIGHT));
	// </FS> Changed for grid manager

	menu_bar_holder->addChild(gMenuBarView);
  
    gViewerWindow->setMenuBackgroundColor(false, 
        !LLGridManager::getInstance()->isInSLBeta());
// <FS:AW opensim currency support>
//	// Assume L$10 for now, the server will tell us the real cost at login
//	// *TODO:Also fix cost in llfolderview.cpp for Inventory menus
//	const std::string upload_cost("10");
	// \0/ Copypasta! See llviewermessage, llviewermenu and llpanelmaininventory
	S32 cost = LLGlobalEconomy::getInstance()->getPriceUpload();
	std::string upload_cost;
#ifdef OPENSIM
	if (LLGridManager::getInstance()->isInOpenSim())
	{
		upload_cost = cost > 0 ? llformat("%s%d", "L$", cost) : LLTrans::getString("free");
	}
	else
#endif
	{
		upload_cost = "L$" + (cost > 0 ? llformat("%d", cost) : llformat("%d", gSavedSettings.getU32("DefaultUploadCost")));
	}
// </FS:AW opensim currency support>
	gMenuHolder->childSetLabelArg("Upload Image", "[COST]", upload_cost);
	gMenuHolder->childSetLabelArg("Upload Sound", "[COST]", upload_cost);
	gMenuHolder->childSetLabelArg("Upload Animation", "[COST]", upload_cost);
	gMenuHolder->childSetLabelArg("Bulk Upload", "[COST]", upload_cost);
	
	gAutorespondMenu = gMenuBarView->getChild<LLMenuItemCallGL>("Set Autorespond", TRUE);
	gAutorespondNonFriendsMenu = gMenuBarView->getChild<LLMenuItemCallGL>("Set Autorespond to non-friends", TRUE);
	gAttachSubMenu = gMenuBarView->findChildMenuByName("Attach Object", TRUE);
	gDetachSubMenu = gMenuBarView->findChildMenuByName("Detach Object", TRUE);

	// Don't display the Memory console menu if the feature is turned off
	LLMenuItemCheckGL *memoryMenu = gMenuBarView->getChild<LLMenuItemCheckGL>("Memory", TRUE);
	if (memoryMenu)
	{
		memoryMenu->setVisible(FALSE);
	}

	gMenuBarView->createJumpKeys();

	// Let land based option enable when parcel changes
	gMenuParcelObserver = new LLMenuParcelObserver();

	gLoginMenuBarView = LLUICtrlFactory::getInstance()->createFromFile<LLMenuBarGL>("menu_login.xml", gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());
	gLoginMenuBarView->arrangeAndClear();
	LLRect menuBarRect = gLoginMenuBarView->getRect();
	menuBarRect.setLeftTopAndSize(0, menu_bar_holder->getRect().getHeight(), menuBarRect.getWidth(), menuBarRect.getHeight());
	gLoginMenuBarView->setRect(menuBarRect);
	// do not set colors in code, always lat the skin decide. -Zi
	// gLoginMenuBarView->setBackgroundColor( color );
	menu_bar_holder->addChild(gLoginMenuBarView);
	
	// tooltips are on top of EVERYTHING, including menus
	gViewerWindow->getRootView()->sendChildToFront(gToolTipView);
}

///////////////////
// SHOW CONSOLES //
///////////////////


class LLAdvancedToggleConsole : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string console_type = userdata.asString();
		if ("texture" == console_type)
		{
			toggle_visibility( (void*)gTextureView );
		}
		else if ("debug" == console_type)
		{
			toggle_visibility( (void*)static_cast<LLUICtrl*>(gDebugView->mDebugConsolep));
		}
		else if ("fast timers" == console_type)
		{
			LLFloaterReg::toggleInstance("block_timers");
		}
		else if ("scene view" == console_type)
		{
			toggle_visibility( (void*)gSceneView);
		}
		else if ("scene monitor" == console_type)
		{
			toggle_visibility( (void*)gSceneMonitorView);
		}

		return true;
	}
};
class LLAdvancedCheckConsole : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string console_type = userdata.asString();
		bool new_value = false;
		if ("texture" == console_type)
		{
			new_value = get_visibility( (void*)gTextureView );
		}
		else if ("debug" == console_type)
		{
			new_value = get_visibility( (void*)((LLView*)gDebugView->mDebugConsolep) );
		}
		else if ("fast timers" == console_type)
		{
			new_value = LLFloaterReg::instanceVisible("block_timers");
		}
		else if ("scene view" == console_type)
		{
			new_value = get_visibility( (void*) gSceneView);
		}
		else if ("scene monitor" == console_type)
		{
			new_value = get_visibility( (void*) gSceneMonitorView);
		}
		
		return new_value;
	}
};


//////////////////////////
// DUMP INFO TO CONSOLE //
//////////////////////////


class LLAdvancedDumpInfoToConsole : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gDebugView->mDebugConsolep->setVisible(TRUE);
		std::string info_type = userdata.asString();
		if ("region" == info_type)
		{
			handle_region_dump_settings(NULL);
		}
		else if ("group" == info_type)
		{
			handle_dump_group_info(NULL);
		}
		else if ("capabilities" == info_type)
		{
			handle_dump_capabilities_info(NULL);
		}
		return true;
	}
};


//////////////
// HUD INFO //
//////////////


class LLAdvancedToggleHUDInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string info_type = userdata.asString();

		if ("camera" == info_type)
		{
			gDisplayCameraPos = !(gDisplayCameraPos);
		}
		else if ("wind" == info_type)
		{
			gDisplayWindInfo = !(gDisplayWindInfo);
		}
		else if ("fov" == info_type)
		{
			gDisplayFOV = !(gDisplayFOV);
		}
		else if ("badge" == info_type)
		{
			report_to_nearby_chat("Hippos!");
		}
		else if ("cookies" == info_type)
		{
			report_to_nearby_chat("Cookies!");
		}
		// <FS:PP>
		else if ("motd" == info_type)
		{
			report_to_nearby_chat(gAgent.mMOTD);
		}
		// </FS:PP>
		return true;
	}
};

class LLAdvancedCheckHUDInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string info_type = userdata.asString();
		bool new_value = false;
		if ("camera" == info_type)
		{
			new_value = gDisplayCameraPos;
		}
		else if ("wind" == info_type)
		{
			new_value = gDisplayWindInfo;
		}
		else if ("fov" == info_type)
		{
			new_value = gDisplayFOV;
		}
		return new_value;
	}
};


//////////////
// FLYING   //
//////////////

class LLAdvancedAgentFlyingInfo : public view_listener_t
{
	bool handleEvent(const LLSD&)
	{
		return gAgent.getFlying();
	}
};


///////////////////////
// CLEAR GROUP CACHE //
///////////////////////

class LLAdvancedClearGroupCache : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLGroupMgr::debugClearAllGroups(NULL);
		return true;
	}
};




/////////////////
// RENDER TYPE //
/////////////////
U32 render_type_from_string(std::string render_type)
{
	if ("simple" == render_type)
	{
		return LLPipeline::RENDER_TYPE_SIMPLE;
	}
	else if ("alpha" == render_type)
	{
		return LLPipeline::RENDER_TYPE_ALPHA;
	}
	else if ("tree" == render_type)
	{
		return LLPipeline::RENDER_TYPE_TREE;
	}
	else if ("character" == render_type)
	{
		return LLPipeline::RENDER_TYPE_AVATAR;
	}
	else if ("surfacePatch" == render_type)
	{
		return LLPipeline::RENDER_TYPE_TERRAIN;
	}
	else if ("sky" == render_type)
	{
		return LLPipeline::RENDER_TYPE_SKY;
	}
	else if ("water" == render_type)
	{
		return LLPipeline::RENDER_TYPE_WATER;
	}
	else if ("ground" == render_type)
	{
		return LLPipeline::RENDER_TYPE_GROUND;
	}
	else if ("volume" == render_type)
	{
		return LLPipeline::RENDER_TYPE_VOLUME;
	}
	else if ("grass" == render_type)
	{
		return LLPipeline::RENDER_TYPE_GRASS;
	}
	else if ("clouds" == render_type)
	{
		return LLPipeline::RENDER_TYPE_CLOUDS;
	}
	else if ("particles" == render_type)
	{
		return LLPipeline::RENDER_TYPE_PARTICLES;
	}
	else if ("bump" == render_type)
	{
		return LLPipeline::RENDER_TYPE_BUMP;
	}
	else
	{
		return 0;
	}
}


class LLAdvancedToggleRenderType : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U32 render_type = render_type_from_string( userdata.asString() );
		if ( render_type != 0 )
		{
			LLPipeline::toggleRenderTypeControl( render_type );
			if(render_type == LLPipeline::RENDER_TYPE_PARTICLES)
			{
				gPipeline.sRenderParticles = gPipeline.hasRenderType(LLPipeline::RENDER_TYPE_PARTICLES);
			}
		}
		return true;
	}
};


class LLAdvancedCheckRenderType : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U32 render_type = render_type_from_string( userdata.asString() );
		bool new_value = false;

		if ( render_type != 0 )
		{
			new_value = LLPipeline::hasRenderTypeControl( render_type );
		}

		return new_value;
	}
};


/////////////
// FEATURE //
/////////////
U32 feature_from_string(std::string feature)
{ 
	if ("ui" == feature)
	{ 
		return LLPipeline::RENDER_DEBUG_FEATURE_UI;
	}
	else if ("selected" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_SELECTED;
	}
	else if ("highlighted" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_HIGHLIGHTED;
	}
	else if ("dynamic textures" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_DYNAMIC_TEXTURES;
	}
	else if ("foot shadows" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_FOOT_SHADOWS;
	}
	else if ("fog" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_FOG;
	}
	else if ("fr info" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_FR_INFO;
	}
	else if ("flexible" == feature)
	{
		return LLPipeline::RENDER_DEBUG_FEATURE_FLEXIBLE;
	}
	else
	{
		return 0;
	}
};


class LLAdvancedToggleFeature : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U32 feature = feature_from_string( userdata.asString() );
		if ( feature != 0 )
		{
			LLPipeline::toggleRenderDebugFeature( feature );
		}
		return true;
	}
};

class LLAdvancedCheckFeature : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
{
	U32 feature = feature_from_string( userdata.asString() );
	bool new_value = false;

	if ( feature != 0 )
	{
		new_value = LLPipeline::toggleRenderDebugFeatureControl( feature );
	}

	return new_value;
}
};

class LLAdvancedCheckDisplayTextureDensity : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string mode = userdata.asString();
		if (!gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY))
		{
			return mode == "none";
		}
		if (mode == "current")
		{
			return LLViewerTexture::sDebugTexelsMode == LLViewerTexture::DEBUG_TEXELS_CURRENT;
		}
		else if (mode == "desired")
		{
			return LLViewerTexture::sDebugTexelsMode == LLViewerTexture::DEBUG_TEXELS_DESIRED;
		}
		else if (mode == "full")
		{
			return LLViewerTexture::sDebugTexelsMode == LLViewerTexture::DEBUG_TEXELS_FULL;
		}
		return false;
	}
};

class LLAdvancedSetDisplayTextureDensity : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string mode = userdata.asString();
		if (mode == "none")
		{
			if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY) == TRUE) 
			{
				gPipeline.toggleRenderDebug(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY);
			}
			LLViewerTexture::sDebugTexelsMode = LLViewerTexture::DEBUG_TEXELS_OFF;
		}
		else if (mode == "current")
		{
			if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY) == FALSE) 
			{
				gPipeline.toggleRenderDebug(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY);
			}
			LLViewerTexture::sDebugTexelsMode = LLViewerTexture::DEBUG_TEXELS_CURRENT;
		}
		else if (mode == "desired")
		{
			if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY) == FALSE) 
			{
				gPipeline.toggleRenderDebug(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY);
			}
			gPipeline.setRenderDebugFeatureControl(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY, true);
			LLViewerTexture::sDebugTexelsMode = LLViewerTexture::DEBUG_TEXELS_DESIRED;
		}
		else if (mode == "full")
		{
			if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY) == FALSE) 
			{
				gPipeline.toggleRenderDebug(LLPipeline::RENDER_DEBUG_TEXEL_DENSITY);
			}
			LLViewerTexture::sDebugTexelsMode = LLViewerTexture::DEBUG_TEXELS_FULL;
		}
		return true;
	}
};


//////////////////
// INFO DISPLAY //
//////////////////
U64 info_display_from_string(std::string info_display)
{
	if ("verify" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_VERIFY;
	}
	else if ("bboxes" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_BBOXES;
	}
	else if ("normals" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_NORMALS;
	}
	else if ("points" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_POINTS;
	}
	else if ("octree" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_OCTREE;
	}
	else if ("shadow frusta" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_SHADOW_FRUSTA;
	}
	else if ("physics shapes" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_PHYSICS_SHAPES;
	}
	else if ("occlusion" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_OCCLUSION;
	}
	else if ("render batches" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_BATCH_SIZE;
	}
	else if ("update type" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_UPDATE_TYPE;
	}
	else if ("texture anim" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TEXTURE_ANIM;
	}
	else if ("texture priority" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TEXTURE_PRIORITY;
	}
	else if ("texture area" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TEXTURE_AREA;
	}
	else if ("face area" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_FACE_AREA;
	}
	else if ("lod info" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_LOD_INFO;
	}
	else if ("build queue" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_BUILD_QUEUE;
	}
	else if ("lights" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_LIGHTS;
	}
	else if ("particles" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_PARTICLES;
	}
	else if ("composition" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_COMPOSITION;
	}
	else if ("avatardrawinfo" == info_display)
	{
		return (LLPipeline::RENDER_DEBUG_AVATAR_DRAW_INFO);
	}
	else if ("glow" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_GLOW;
	}
	else if ("collision skeleton" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_AVATAR_VOLUME;
	}
	else if ("joints" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_AVATAR_JOINTS;
	}
	else if ("raycast" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_RAYCAST;
	}
	else if ("agent target" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_AGENT_TARGET;
	}
	else if ("sculpt" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_SCULPTED;
	}
	else if ("wind vectors" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_WIND_VECTORS;
	}
	else if ("texel density" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TEXEL_DENSITY;
	}
	else if ("triangle count" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TRIANGLE_COUNT;
	}
	else if ("impostors" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_IMPOSTORS;
	}
	else if ("texture size" == info_display)
	{
		return LLPipeline::RENDER_DEBUG_TEXTURE_SIZE;
	}
	else
	{
		LL_WARNS() << "unrecognized feature name '" << info_display << "'" << LL_ENDL;
		return 0;
	}
};

class LLAdvancedToggleInfoDisplay : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U64 info_display = info_display_from_string( userdata.asString() );

		LL_INFOS("ViewerMenu") << "toggle " << userdata.asString() << LL_ENDL;
		
		if ( info_display != 0 )
		{
			LLPipeline::toggleRenderDebug( info_display );
		}

		return true;
	}
};


class LLAdvancedCheckInfoDisplay : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U64 info_display = info_display_from_string( userdata.asString() );
		bool new_value = false;

		if ( info_display != 0 )
		{
			new_value = LLPipeline::toggleRenderDebugControl( info_display );
		}

		return new_value;
	}
};


///////////////////////////
//// RANDOMIZE FRAMERATE //
///////////////////////////


class LLAdvancedToggleRandomizeFramerate : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gRandomizeFramerate = !(gRandomizeFramerate);
		return true;
	}
};

class LLAdvancedCheckRandomizeFramerate : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gRandomizeFramerate;
		return new_value;
	}
};

///////////////////////////
//// PERIODIC SLOW FRAME //
///////////////////////////


class LLAdvancedTogglePeriodicSlowFrame : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gPeriodicSlowFrame = !(gPeriodicSlowFrame);
		return true;
	}
};

class LLAdvancedCheckPeriodicSlowFrame : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gPeriodicSlowFrame;
		return new_value;
	}
};



////////////////
// FRAME TEST //
////////////////


class LLAdvancedToggleFrameTest : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLPipeline::sRenderFrameTest = !(LLPipeline::sRenderFrameTest);
		return true;
	}
};

class LLAdvancedCheckFrameTest : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLPipeline::sRenderFrameTest;
		return new_value;
	}
};


///////////////////////////
// SELECTED TEXTURE INFO //
///////////////////////////


class LLAdvancedSelectedTextureInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_selected_texture_info(NULL);
		return true;
	}
};

//////////////////////
// TOGGLE WIREFRAME //
//////////////////////

class LLAdvancedToggleWireframe : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: RLVa-2.0.0
		bool fRlvBlockWireframe = gRlvAttachmentLocks.hasLockedHUD();
		if ( (!gUseWireframe) && (fRlvBlockWireframe) )
			RlvUtil::notifyBlocked(RLV_STRING_BLOCKED_WIREFRAME);
		set_use_wireframe( (!gUseWireframe) && (!fRlvBlockWireframe) );
		return true;
	}
};

// Called from rlvhandler.cpp
void set_use_wireframe(bool useWireframe)
	{
		if (gUseWireframe == useWireframe)
			return;

		gUseWireframe = useWireframe;
// [/RLVa:KB]
//		gUseWireframe = !(gUseWireframe);
		gWindowResized = TRUE;

		LLPipeline::updateRenderDeferred();

		if (gUseWireframe)
		{
			gInitialDeferredModeForWireframe = LLPipeline::sRenderDeferred;
		}

		gPipeline.resetVertexBuffers();

		if (!gUseWireframe && !gInitialDeferredModeForWireframe && LLPipeline::sRenderDeferred != bool(gInitialDeferredModeForWireframe) && gPipeline.isInit())
		{
			LLPipeline::refreshCachedSettings();
			gPipeline.releaseGLBuffers();
			gPipeline.createGLBuffers();
			LLViewerShaderMgr::instance()->setShaders();
		}

//		return true;
	}
//};

class LLAdvancedCheckWireframe : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gUseWireframe;
		return new_value;
	}
};
	

//////////////////////////
// DUMP SCRIPTED CAMERA //
//////////////////////////
	
class LLAdvancedDumpScriptedCamera : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_dump_followcam(NULL);
		return true;
	}
};



//////////////////////////////
// DUMP REGION OBJECT CACHE //
//////////////////////////////


class LLAdvancedDumpRegionObjectCache : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
{
		handle_dump_region_object_cache(NULL);
		return true;
	}
};

class LLAdvancedBuyCurrencyTest : public view_listener_t
	{
	bool handleEvent(const LLSD& userdata)
	{
		handle_buy_currency_test(NULL);
		return true;
	}
};


/////////////////////
// DUMP SELECT MGR //
/////////////////////


class LLAdvancedDumpSelectMgr : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		dump_select_mgr(NULL);
		return true;
	}
};



////////////////////
// DUMP INVENTORY //
////////////////////


class LLAdvancedDumpInventory : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		dump_inventory(NULL);
		return true;
	}
};



////////////////////////////////
// PRINT SELECTED OBJECT INFO //
////////////////////////////////


class LLAdvancedPrintSelectedObjectInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		print_object_info(NULL);
		return true;
	}
};



//////////////////////
// PRINT AGENT INFO //
//////////////////////


class LLAdvancedPrintAgentInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		print_agent_nvpairs(NULL);
		return true;
	}
};

//////////////////
// DEBUG CLICKS //
//////////////////


class LLAdvancedToggleDebugClicks : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gDebugClicks = !(gDebugClicks);
		return true;
	}
};

class LLAdvancedCheckDebugClicks : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gDebugClicks;
		return new_value;
	}
};



/////////////////
// DEBUG VIEWS //
/////////////////


class LLAdvancedToggleDebugViews : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLView::sDebugRects = !(LLView::sDebugRects);
		return true;
	}
};

class LLAdvancedCheckDebugViews : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLView::sDebugRects;
		return new_value;
	}
};



///////////////////////
// XUI NAME TOOLTIPS //
///////////////////////


class LLAdvancedToggleXUINameTooltips : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		toggle_show_xui_names(NULL);
		return true;
	}
};

class LLAdvancedCheckXUINameTooltips : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = check_show_xui_names(NULL);
		return new_value;
	}
};



////////////////////////
// DEBUG MOUSE EVENTS //
////////////////////////


class LLAdvancedToggleDebugMouseEvents : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLView::sDebugMouseHandling = !(LLView::sDebugMouseHandling);
		return true;
	}
};

class LLAdvancedCheckDebugMouseEvents : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLView::sDebugMouseHandling;
		return new_value;
	}
};



////////////////
// DEBUG KEYS //
////////////////


class LLAdvancedToggleDebugKeys : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLView::sDebugKeys = !(LLView::sDebugKeys);
		return true;
	}
};
	
class LLAdvancedCheckDebugKeys : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLView::sDebugKeys;
		return new_value;
	}
};
	


///////////////////////
// DEBUG WINDOW PROC //
///////////////////////


class LLAdvancedToggleDebugWindowProc : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gDebugWindowProc = !(gDebugWindowProc);
		return true;
	}
};

class LLAdvancedCheckDebugWindowProc : public view_listener_t
	{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gDebugWindowProc;
		return new_value;
	}
};

// ------------------------------XUI MENU ---------------------------

//////////////////////
// LOAD UI FROM XML //
//////////////////////


class LLAdvancedLoadUIFromXML : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_load_from_xml(NULL);
		return true;
	}
};



////////////////////
// SAVE UI TO XML //
////////////////////


class LLAdvancedSaveUIToXML : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_save_to_xml(NULL);
		return true;
	}
};


class LLAdvancedSendTestIms : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLIMModel::instance().testMessages();
		return true;
	}
};


///////////////
// XUI NAMES //
///////////////


class LLAdvancedToggleXUINames : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		toggle_show_xui_names(NULL);
		return true;
	}
};

class LLAdvancedCheckXUINames : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = check_show_xui_names(NULL);
		return new_value;
	}
};


////////////////////////
// GRAB BAKED TEXTURE //
////////////////////////


class LLAdvancedGrabBakedTexture : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string texture_type = userdata.asString();
		if ("iris" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_EYES );
		}
		else if ("head" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_HEAD );
		}
		else if ("upper" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_UPPER );
		}
		else if ("lower" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_LOWER );
		}
		else if ("skirt" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_SKIRT );
		}
		else if ("hair" == texture_type)
		{
			handle_grab_baked_texture( (void*)BAKED_HAIR );
		}

		return true;
	}
};

class LLAdvancedEnableGrabBakedTexture : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
{
		std::string texture_type = userdata.asString();
		bool new_value = false;

		if ("iris" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_EYES );
		}
		else if ("head" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_HEAD );
		}
		else if ("upper" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_UPPER );
		}
		else if ("lower" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_LOWER );
		}
		else if ("skirt" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_SKIRT );
		}
		else if ("hair" == texture_type)
		{
			new_value = enable_grab_baked_texture( (void*)BAKED_HAIR );
		}
	
		return new_value;
}
};

///////////////////////
// APPEARANCE TO XML //
///////////////////////


class LLAdvancedEnableAppearanceToXML : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
        LLViewerObject *obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
        if (obj && obj->isAnimatedObject() && obj->getControlAvatar())
        {
            return gSavedSettings.getBOOL("DebugAnimatedObjects");
        }
        else if (obj && obj->isAttachment() && obj->getAvatar())
        {
            return gSavedSettings.getBOOL("DebugAvatarAppearanceMessage");
        }
        else if (obj && obj->isAvatar())
        {
            // This has to be a non-control avatar, because control avs are invisible and unclickable.
            return gSavedSettings.getBOOL("DebugAvatarAppearanceMessage");
        }
		else
		{
			return false;
		}
	}
};

class LLAdvancedAppearanceToXML : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string emptyname;
        LLViewerObject *obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
        LLVOAvatar *avatar = NULL;
        if (obj)
        {
            if (obj->isAvatar())
            {
                avatar = obj->asAvatar();
            }
            else
            {
                // If there is a selection, find the associated
                // avatar. Normally there's only one obvious choice. But
                // what should be returned if the object is in an attached
                // animated object? getAvatar() will give the skeleton of
                // the animated object. getAvatarAncestor() will give the
                // actual human-driven avatar.
                avatar = obj->getAvatar();
            }
        }
        else
        {
            // If no selection, use the self avatar.
			avatar = gAgentAvatarp;
        }
        if (avatar)
        {
            avatar->dumpArchetypeXML(emptyname);
        }
		return true;
	}
};



///////////////////////////////
// TOGGLE CHARACTER GEOMETRY //
///////////////////////////////


class LLAdvancedToggleCharacterGeometry : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_god_request_avatar_geometry(NULL);
		return true;
	}
};


	/////////////////////////////
// TEST MALE / TEST FEMALE //
/////////////////////////////

class LLAdvancedTestMale : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_test_male(NULL);
		return true;
	}
};


class LLAdvancedTestFemale : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_test_female(NULL);
		return true;
	}
};

class LLAdvancedForceParamsToDefault : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLAgent::clearVisualParams(NULL);
		return true;
	}
};


//////////////////////////
//   ANIMATION SPEED    //
//////////////////////////

// Utility function to set all AV time factors to the same global value
static void set_all_animation_time_factors(F32	time_factor)
{
	LLMotionController::setCurrentTimeFactor(time_factor);
	for (std::vector<LLCharacter*>::iterator iter = LLCharacter::sInstances.begin();
		iter != LLCharacter::sInstances.end(); ++iter)
	{
		(*iter)->setAnimTimeFactor(time_factor);
	}
}

class LLAdvancedAnimTenFaster : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		//LL_INFOS() << "LLAdvancedAnimTenFaster" << LL_ENDL;
		F32 time_factor = LLMotionController::getCurrentTimeFactor();
		time_factor = llmin(time_factor + 0.1f, 2.f);	// Upper limit is 200% speed
		set_all_animation_time_factors(time_factor);
		return true;
	}
};

class LLAdvancedAnimTenSlower : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		//LL_INFOS() << "LLAdvancedAnimTenSlower" << LL_ENDL;
		F32 time_factor = LLMotionController::getCurrentTimeFactor();
		time_factor = llmax(time_factor - 0.1f, 0.1f);	// Lower limit is at 10% of normal speed
		set_all_animation_time_factors(time_factor);
		return true;
	}
};

class LLAdvancedAnimResetAll : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		set_all_animation_time_factors(1.f);
		return true;
	}
};


//////////////////////////
// RELOAD VERTEX SHADER //
//////////////////////////


class LLAdvancedReloadVertexShader : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		reload_vertex_shader(NULL);
		return true;
	}
};



////////////////////
// ANIMATION INFO //
////////////////////


class LLAdvancedToggleAnimationInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar::sShowAnimationDebug = !(LLVOAvatar::sShowAnimationDebug);
		return true;
	}
};

class LLAdvancedCheckAnimationInfo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLVOAvatar::sShowAnimationDebug;
		return new_value;
	}
};


//////////////////
// SHOW LOOK AT //
//////////////////


class LLAdvancedToggleShowLookAt : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		//LLHUDEffectLookAt::sDebugLookAt = !(LLHUDEffectLookAt::sDebugLookAt);
		//<FS:AO improve use of controls with radiogroups>
		//bool value = !gSavedPerAccountSettings.getBOOL("DebugLookAt");
		//gSavedPerAccountSettings.setBOOL("DebugLookAt",value);
		S32 value = !gSavedPerAccountSettings.getS32("DebugLookAt");
		gSavedPerAccountSettings.setS32("DebugLookAt",value);
		//</FS:AO>
		return true;
	}
};

// <AO>
class LLAdvancedToggleShowColor : public view_listener_t
{
        bool handleEvent(const LLSD& userdata)
        {
                S32 value = !gSavedSettings.getS32("DebugShowColor");
                gSavedSettings.setS32("DebugShowColor",value);
                return true;
        }
};

class LLAdvancedCheckShowColor : public view_listener_t
{
        bool handleEvent(const LLSD& userdata)
        {
                S32 new_value = gSavedSettings.getS32("DebugShowColor");
                return (bool)new_value;
        }
};
// </AO>

class LLAdvancedCheckShowLookAt : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		//bool new_value = LLHUDEffectLookAt::sDebugLookAt;
		//<FS:AO improve use of controls with radiogroups>
		//bool new_value = gSavedPerAccountSettings.getBOOL("DebugLookAt");
		S32 new_value = gSavedPerAccountSettings.getS32("DebugLookAt");
		return (bool)new_value;
	}
};



///////////////////
// SHOW POINT AT //
///////////////////


class LLAdvancedToggleShowPointAt : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLHUDEffectPointAt::sDebugPointAt = !(LLHUDEffectPointAt::sDebugPointAt);
		return true;
	}
};

class LLAdvancedCheckShowPointAt : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLHUDEffectPointAt::sDebugPointAt;
		return new_value;
	}
};


///////////////////// 
// PRIVATE LOOK AT // 
///////////////////// 

class LLAdvancedTogglePrivateLookPointAt : public view_listener_t 
{ 
	bool handleEvent(const LLSD& userdata) 
	{ 
		std::string command = userdata.asString(); 
		if ("Look" == command) 
		{ 
			bool new_value = !gSavedSettings.getBOOL("PrivateLookAtTarget"); 
			gSavedSettings.setBOOL("PrivateLookAtTarget", new_value); 
		} 
		else if ("Point" == command) 
		{ 
			bool new_value = !gSavedSettings.getBOOL("PrivatePointAtTarget"); 
			gSavedSettings.setBOOL("PrivatePointAtTarget", new_value); 
		} 
	return true; 
	} 
}; 

class LLAdvancedCheckPrivateLookPointAt : public view_listener_t 
{ 
	bool handleEvent(const LLSD& userdata) 
	{ 
		std::string command = userdata["data"].asString(); 
		if ("Look" == command) 
		{ 
			bool new_value = gSavedSettings.getBOOL("PrivateLookAtTarget"); 
			std::string control_name = userdata["control"].asString(); 
			gMenuHolder->findControl(control_name)->setValue(new_value); 
		} 
		else if ("Point" == command) 
		{ 
			bool new_value = gSavedSettings.getBOOL("PrivatePointAtTarget"); 
			std::string control_name = userdata["control"].asString(); 
			gMenuHolder->findControl(control_name)->setValue(new_value); 
		} 
	return true; 
	} 
};

/////////////////////////
// DEBUG JOINT UPDATES //
/////////////////////////


class LLAdvancedToggleDebugJointUpdates : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar::sJointDebug = !(LLVOAvatar::sJointDebug);
		return true;
	}
};

class LLAdvancedCheckDebugJointUpdates : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLVOAvatar::sJointDebug;
		return new_value;
	}
};



/////////////////
// DISABLE LOD //
/////////////////


class LLAdvancedToggleDisableLOD : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerJoint::sDisableLOD = !(LLViewerJoint::sDisableLOD);
		return true;
	}
};
		
class LLAdvancedCheckDisableLOD : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLViewerJoint::sDisableLOD;
		return new_value;
	}
};



/////////////////////////
// DEBUG CHARACTER VIS //
/////////////////////////


class LLAdvancedToggleDebugCharacterVis : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar::sDebugInvisible = !(LLVOAvatar::sDebugInvisible);
		return true;
	}
};

class LLAdvancedCheckDebugCharacterVis : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLVOAvatar::sDebugInvisible;
		return new_value;
	}
};


//////////////////////
// DUMP ATTACHMENTS //
//////////////////////

	
class LLAdvancedDumpAttachments : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_dump_attachments(NULL);
		return true;
	}
};


	
/////////////////////
// REBAKE TEXTURES //
/////////////////////
	
	
class LLAdvancedRebakeTextures : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_rebake_textures(NULL);
		return true;
	}
};
	
	
// [SL:KB] - Patch: Appearance-PhantomAttach | Checked: Catznip-5.0
void handle_refresh_attachments()
{
	LLAttachmentsMgr::instance().refreshAttachments();
}
// [/SL:KB]

#if 1 //ndef LL_RELEASE_FOR_DOWNLOAD
///////////////////////////
// DEBUG AVATAR TEXTURES //
///////////////////////////


class LLAdvancedDebugAvatarTextures : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.isGodlike())
		{
			handle_debug_avatar_textures(NULL);
		}
		return true;
	}
};

////////////////////////////////
// DUMP AVATAR LOCAL TEXTURES //
////////////////////////////////


class LLAdvancedDumpAvatarLocalTextures : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
#ifndef LL_RELEASE_FOR_DOWNLOAD
		handle_dump_avatar_local_textures(NULL);
#endif
		return true;
	}
};

#endif

///////////////////////////////////
// Reload Avatar Cloud Particles //
///////////////////////////////////
class LLAdvancedReloadAvatarCloudParticle : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar::initCloud();
		return true;
	}
};

/////////////////
// MESSAGE LOG //
/////////////////


class LLAdvancedEnableMessageLog : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_viewer_enable_message_log(NULL);
		return true;
	}
};

class LLAdvancedDisableMessageLog : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_viewer_disable_message_log(NULL);
		return true;
	}
};

/////////////////
// DROP PACKET //
/////////////////


class LLAdvancedDropPacket : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gMessageSystem->mPacketRing.dropPackets(1);
		return true;
	}
};


////////////////////
// EVENT Recorder //
///////////////////


class LLAdvancedViewerEventRecorder : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string command = userdata.asString();
		if ("start playback" == command)
		{
			LL_INFOS() << "Event Playback starting" << LL_ENDL;
			LLViewerEventRecorder::instance().playbackRecording();
			LL_INFOS() << "Event Playback completed" << LL_ENDL;
		}
		else if ("stop playback" == command)
		{
			// Future
		}
		else if ("start recording" == command)
		{
			LLViewerEventRecorder::instance().setEventLoggingOn();
			LL_INFOS() << "Event recording started" << LL_ENDL;
		}
		else if ("stop recording" == command)
		{
			LLViewerEventRecorder::instance().setEventLoggingOff();
			LL_INFOS() << "Event recording stopped" << LL_ENDL;
		} 

		return true;
	}		
};




/////////////////
// AGENT PILOT //
/////////////////


class LLAdvancedAgentPilot : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string command = userdata.asString();
		if ("start playback" == command)
		{
			gAgentPilot.setNumRuns(-1);
			gAgentPilot.startPlayback();
		}
		else if ("stop playback" == command)
		{
			gAgentPilot.stopPlayback();
		}
		else if ("start record" == command)
		{
			gAgentPilot.startRecord();
		}
		else if ("stop record" == command)
		{
			gAgentPilot.stopRecord();
		}

		return true;
	}		
};



//////////////////////
// AGENT PILOT LOOP //
//////////////////////


class LLAdvancedToggleAgentPilotLoop : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gAgentPilot.setLoop(!gAgentPilot.getLoop());
		return true;
	}
};

class LLAdvancedCheckAgentPilotLoop : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgentPilot.getLoop();
		return new_value;
	}
};


/////////////////////////
// SHOW OBJECT UPDATES //
/////////////////////////


class LLAdvancedToggleShowObjectUpdates : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gShowObjectUpdates = !(gShowObjectUpdates);
		return true;
	}
};

class LLAdvancedCheckShowObjectUpdates : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gShowObjectUpdates;
		return new_value;
	}
};



////////////////////
// COMPRESS IMAGE //
////////////////////


class LLAdvancedCompressImage : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_compress_image(NULL);
		return true;
	}
};


/////////////////////////
// SHOW DEBUG SETTINGS //
/////////////////////////


class LLAdvancedShowDebugSettings : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFloaterReg::showInstance("settings_debug",userdata);
		return true;
	}
};



////////////////////////
// VIEW ADMIN OPTIONS //
////////////////////////

class LLAdvancedEnableViewAdminOptions : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// Don't enable in god mode since the admin menu is shown anyway.
		// Only enable if the user has set the appropriate debug setting.
		bool new_value = !gAgent.getAgentAccess().isGodlikeWithoutAdminMenuFakery() && gSavedSettings.getBOOL("AdminMenu");
		return new_value;
	}
};

class LLAdvancedToggleViewAdminOptions : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_admin_override_toggle(NULL);
		return true;
	}
};

class LLAdvancedToggleVisualLeakDetector : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_visual_leak_detector_toggle(NULL);
		return true;
	}
};

class LLAdvancedCheckViewAdminOptions : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = check_admin_override(NULL) || gAgent.isGodlike();
		return new_value;
	}
};

/////////////////////////////////////
// Enable Object Object Occlusion ///
/////////////////////////////////////
class LLAdvancedEnableObjectObjectOcclusion: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
	
		bool new_value = gGLManager.mHasOcclusionQuery; // && LLFeatureManager::getInstance()->isFeatureAvailable(userdata.asString());
		return new_value;
}
};

/////////////////////////////////////
// Enable Framebuffer Objects	  ///
/////////////////////////////////////
class LLAdvancedEnableRenderFBO: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gGLManager.mHasFramebufferObject;
		return new_value;
	}
};

/////////////////////////////////////
// Enable Advanced Lighting Model ///
/////////////////////////////////////
class LLAdvancedEnableRenderDeferred: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gGLManager.mHasFramebufferObject && LLViewerShaderMgr::instance()->getVertexShaderLevel(LLViewerShaderMgr::SHADER_WINDLIGHT) > 1 &&
			LLViewerShaderMgr::instance()->getVertexShaderLevel(LLViewerShaderMgr::SHADER_AVATAR) > 0;
		return new_value;
	}
};

/////////////////////////////////////
// Enable Advanced Lighting Model sub-options
/////////////////////////////////////
class LLAdvancedEnableRenderDeferredOptions: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gGLManager.mHasFramebufferObject && LLViewerShaderMgr::instance()->getVertexShaderLevel(LLViewerShaderMgr::SHADER_WINDLIGHT) > 1 &&
			LLViewerShaderMgr::instance()->getVertexShaderLevel(LLViewerShaderMgr::SHADER_AVATAR) > 0 && gSavedSettings.getBOOL("RenderDeferred");
		return new_value;
	}
};



//////////////////
// ADMIN STATUS //
//////////////////


class LLAdvancedRequestAdminStatus : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_god_mode(NULL);
		return true;
	}
};

class LLAdvancedLeaveAdminStatus : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_leave_god_mode(NULL);
		return true;
	}
};

//////////////////////////
// Advanced > Debugging //
//////////////////////////


class LLAdvancedForceErrorBreakpoint : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_breakpoint(NULL);
		return true;
	}
};

class LLAdvancedForceErrorLlerror : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_llerror(NULL);
		return true;
	}
};
class LLAdvancedForceErrorBadMemoryAccess : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_bad_memory_access(NULL);
		return true;
	}
};

class LLAdvancedForceErrorInfiniteLoop : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_infinite_loop(NULL);
		return true;
	}
};

class LLAdvancedForceErrorSoftwareException : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_software_exception(NULL);
		return true;
	}
};

class LLAdvancedForceErrorDriverCrash : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_error_driver_crash(NULL);
		return true;
	}
};

class LLAdvancedForceErrorDisconnectViewer : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_disconnect_viewer(NULL);
		return true;
}
};


#ifdef TOGGLE_HACKED_GODLIKE_VIEWER

class LLAdvancedHandleToggleHackedGodmode : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_toggle_hacked_godmode(NULL);
		return true;
	}
};

class LLAdvancedCheckToggleHackedGodmode : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		check_toggle_hacked_godmode(NULL);
		return true;
	}
};

class LLAdvancedEnableToggleHackedGodmode : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = enable_toggle_hacked_godmode(NULL);
		return new_value;
	}
};
#endif


//
////-------------------------------------------------------------------
//// Advanced menu
////-------------------------------------------------------------------


//////////////////
// DEVELOP MENU //
//////////////////

class LLDevelopCheckLoggingLevel : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U32 level = userdata.asInteger();
		return (static_cast<LLError::ELevel>(level) == LLError::getDefaultLevel());
	}
};

class LLDevelopSetLoggingLevel : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		U32 level = userdata.asInteger();
		LLError::setDefaultLevel(static_cast<LLError::ELevel>(level));
		return true;
	}
};

class LLDevelopTextureFetchDebugger : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return gSavedSettings.getBOOL("TextureFetchDebuggerEnabled");
	}
};

//////////////////
// ADMIN MENU   //
//////////////////

// Admin > Object
class LLAdminForceTakeCopy : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		force_take_copy(NULL);
		return true;
	}
};

class LLAdminHandleObjectOwnerSelf : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_object_owner_self(NULL);
		return true;
	}
};
class LLAdminHandleObjectOwnerPermissive : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_object_owner_permissive(NULL);
		return true;
	}
};

class LLAdminHandleForceDelete : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_force_delete(NULL);
		return true;
	}
};

class LLAdminHandleObjectLock : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_object_lock(NULL);
		return true;
	}
};

class LLAdminHandleObjectAssetIDs: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_object_asset_ids(NULL);
		return true;
	}	
};

//Admin >Parcel
class LLAdminHandleForceParcelOwnerToMe: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_force_parcel_owner_to_me(NULL);
		return true;
	}
};
class LLAdminHandleForceParcelToContent: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_force_parcel_to_content(NULL);
		return true;
	}
};
class LLAdminHandleClaimPublicLand: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_claim_public_land(NULL);
		return true;
	}
};

// Admin > Region
class LLAdminHandleRegionDumpTempAssetData: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_region_dump_temp_asset_data(NULL);
		return true;
	}
};
//Admin (Top Level)

class LLAdminOnSaveState: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLPanelRegionTools::onSaveState(NULL);
		return true;
}
};


//-----------------------------------------------------------------------------
// cleanup_menus()
//-----------------------------------------------------------------------------
void cleanup_menus()
{
	delete gMenuParcelObserver;
	gMenuParcelObserver = NULL;

	delete gMenuAvatarSelf;
	gMenuAvatarSelf = NULL;

	delete gMenuAvatarOther;
	gMenuAvatarOther = NULL;

	delete gMenuObject;
	gMenuObject = NULL;

	delete gMenuAttachmentSelf;
	gMenuAttachmentSelf = NULL;

	delete gMenuAttachmentOther;
	gMenuAttachmentSelf = NULL;

	delete gMenuLand;
	gMenuLand = NULL;

	delete gMenuMuteParticle;
	gMenuMuteParticle = NULL;

	// <FS:Ansariel> Pie menu
	delete gPieMenuAvatarSelf;
	gPieMenuAvatarSelf = NULL;

	delete gPieMenuAvatarOther;
	gPieMenuAvatarOther = NULL;

	delete gPieMenuObject;
	gPieMenuObject = NULL;

	delete gPieMenuAttachmentSelf;
	gPieMenuAttachmentSelf = NULL;

	delete gPieMenuAttachmentOther;
	gPieMenuAttachmentOther = NULL;

	delete gPieMenuLand;
	gPieMenuLand = NULL;

	delete gPieMenuMuteParticle;
	gPieMenuMuteParticle = NULL;
	// </FS:Ansariel>

	delete gMenuBarView;
	gMenuBarView = NULL;

	delete gPopupMenuView;
	gPopupMenuView = NULL;

	delete gMenuHolder;
	gMenuHolder = NULL;
}

//-----------------------------------------------------------------------------
// Object pie menu
//-----------------------------------------------------------------------------

// <FS:Ansariel> FIRE-6970/FIRE-6998: Optional permanent derendering of multiple objects
void derenderObject(bool permanent)
{
	bool need_save = false;
	LLViewerObject* objp;
	LLSelectMgr* select_mgr = LLSelectMgr::getInstance();

	while ((objp = select_mgr->getSelection()->getFirstRootObject(TRUE)))
	{
//		if ( (objp) && (gAgentID != objp->getID()) )
// [RLVa:KB] - Checked: 2012-03-11 (RLVa-1.4.5) | Added: RLVa-1.4.5 | FS-specific
		// Don't allow derendering of own attachments when RLVa is enabled
		if ( (objp) && (gAgentID != objp->getID()) && ((!rlv_handler_t::isEnabled()) || (!objp->isAttachment()) || (!objp->permYouOwner())) )
// [/RLVa:KB]
		{
			LLUUID id = objp->getID();
			std::string entry_name = "";
			std::string region_name;
			LLAssetType::EType asset_type;

			if (objp->isAvatar())
			{
				LLNameValue* firstname = objp->getNVPair("FirstName");
				LLNameValue* lastname = objp->getNVPair("LastName");
				entry_name = llformat("%s %s", firstname->getString(), lastname->getString());
				asset_type = LLAssetType::AT_PERSON;
			}
			else
			{
				bool next_object = false;
				LLViewerObject::child_list_t object_children = objp->getChildren();
				for (LLViewerObject::child_list_t::const_iterator it = object_children.begin(); it != object_children.end(); it++)
				{
					LLViewerObject* child = *it;
					if (child->isAvatar() && child->asAvatar()->isSelf())
					{
						if (gRlvHandler.hasBehaviour(RLV_BHVR_UNSIT))
						{
							// RLVa: Prevent cheating out of sitting by derendering the object
							select_mgr->deselectObjectOnly(objp);
							next_object = true;
						}
						else
						{
							gAgent.standUp();
						}
						break;
					}
				}

				if (next_object)
				{
					continue;
				}

				LLSelectNode* nodep = select_mgr->getSelection()->getFirstRootNode();
				if (nodep)
				{
					if (!nodep->mName.empty())
					{
						entry_name = nodep->mName;
					}
				}
				LLViewerRegion* region = objp->getRegion();
				if (region)
				{
					region_name = region->getName();
				}
				asset_type = LLAssetType::AT_OBJECT;
			}
			
			FSAssetBlacklist::getInstance()->addNewItemToBlacklist(id, entry_name, region_name, asset_type, permanent, false);
			
			if (permanent)
			{
				need_save = true;
			}

			select_mgr->deselectObjectOnly(objp);
			gObjectList.addDerenderedItem(id, permanent);
			gObjectList.killObject(objp);
			if (LLViewerRegion::sVOCacheCullingEnabled && objp->getRegion())
			{
				objp->getRegion()->killCacheEntry(objp->getLocalID());
			}

			LLTool* tool = LLToolMgr::getInstance()->getCurrentTool();
			LLViewerObject* tool_editing_object = tool->getEditingObject();
			if (tool_editing_object && tool_editing_object->mID == id)
			{
				tool->stopEditing();
			}

		}
		else if( (objp) && (gAgentID != objp->getID()) && ((rlv_handler_t::isEnabled()) || (objp->isAttachment()) || (objp->permYouOwner())) )
		{
			select_mgr->deselectObjectOnly(objp);
			return;
		}
	}

	if (need_save)
	{
		FSAssetBlacklist::getInstance()->saveBlacklist();
	}
}

class LLObjectDerenderPermanent : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		derenderObject(true);
		return true;
	}
};

class LLObjectDerender : public view_listener_t
{
    bool handleEvent(const LLSD& userdata)
    {
		derenderObject(false);
		return true;
    }
};
// </FS:Ansariel>

// <FS:CR> FIRE-10082 - Don't enable derendering own attachments when RLVa is enabled
bool enable_derender_object()
{
	return (!rlv_handler_t::isEnabled());
}
// </FS:CR>

class LLEnableEditParticleSource : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLObjectSelectionHandle handle = LLSelectMgr::instance().getSelection();

		if (handle->getObjectCount() >= 1)
		{
			LLObjectSelection::valid_iterator iter = handle->valid_begin();
			if (iter == handle->valid_end())
			{
				return false;
			}

			LLSelectNode* node = *iter;

			if (!node || !node->mPermissions)
			{
				return false;
			}

			if (node->mPermissions->getOwner() == gAgentID)
			{
				return true;
			}
		}
		return false;
	}
};

class LLEditParticleSource : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (objectp)
		{
			ParticleEditor* particleEditor = LLFloaterReg::showTypedInstance<ParticleEditor>("particle_editor", LLSD(objectp->getID()), TAKE_FOCUS_YES);
			if (particleEditor)
			{
				particleEditor->setObject(objectp);
			}
		}
		return true;
	}
};

// <FS:Zi> Texture Refresh
void destroy_texture(const LLUUID& id)		// will be used by the texture refresh functions below
{
	if (id.isNull() || id == IMG_DEFAULT || FSCommon::isDefaultTexture(id))
	{
		return;
	}

	LLViewerFetchedTexture* tx = LLViewerTextureManager::getFetchedTexture(id);
	if (tx)
	{
		tx->clearFetchedResults();
	}
	LLAppViewer::getTextureCache()->removeFromCache(id);
}

class LLObjectTexRefresh : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// partly copied from the texture info code in handle_selected_texture_info()
		for (LLObjectSelection::valid_iterator iter = LLSelectMgr::getInstance()->getSelection()->valid_begin();
			iter != LLSelectMgr::getInstance()->getSelection()->valid_end(); iter++)
		{
			LLSelectNode* node = *iter;

			U8 te_count = node->getObject()->getNumTEs();
			// map from texture ID to list of faces using it
			typedef std::map< LLUUID, std::vector<U8> > map_t;
			map_t faces_per_texture;
			for (U8 i = 0; i < te_count; ++i)
			{
				if (!node->isTESelected(i)) continue;

				LLViewerTexture* img = node->getObject()->getTEImage(i);
				faces_per_texture[img->getID()].push_back(i);

				if (node->getObject()->getTE(i)->getMaterialParams().notNull())
				{
					LLViewerTexture* norm_img = node->getObject()->getTENormalMap(i);
					faces_per_texture[norm_img->getID()].push_back(i);

					LLViewerTexture* spec_img = node->getObject()->getTESpecularMap(i);
					faces_per_texture[spec_img->getID()].push_back(i);
				}
			}

			map_t::iterator it;
			for (it = faces_per_texture.begin(); it != faces_per_texture.end(); ++it)
			{
				destroy_texture(it->first);
			}

			// Refresh sculpt texture
			if (node->getObject()->isSculpted())
			{
				LLSculptParams *sculpt_params = (LLSculptParams *)node->getObject()->getParameterEntry(LLNetworkData::PARAMS_SCULPT);
				if (sculpt_params)
				{
					LLUUID sculpt_uuid = sculpt_params->getSculptTexture();

					LLViewerFetchedTexture* tx = LLViewerTextureManager::getFetchedTexture(sculpt_uuid);
					if (tx)
					{
						S32 num_volumes = tx->getNumVolumes(LLRender::SCULPT_TEX);
						const LLViewerTexture::ll_volume_list_t* pVolumeList = tx->getVolumeList(LLRender::SCULPT_TEX);

						destroy_texture(sculpt_uuid);

						for (S32 idxVolume = 0; idxVolume < num_volumes; ++idxVolume)
						{
							LLVOVolume* pVolume = pVolumeList->at(idxVolume);
							if (pVolume)
							{
								pVolume->notifyMeshLoaded();
							}
						}
					}
				}
			}
		}

		return true;
	}
};

void avatar_tex_refresh(LLVOAvatar* avatar)
{
	// I bet this can be done more elegantly, but this is just straightforward
	destroy_texture(avatar->getTE(TEX_HEAD_BAKED)->getID());
	destroy_texture(avatar->getTE(TEX_UPPER_BAKED)->getID());
	destroy_texture(avatar->getTE(TEX_LOWER_BAKED)->getID());
	destroy_texture(avatar->getTE(TEX_EYES_BAKED)->getID());
	destroy_texture(avatar->getTE(TEX_SKIRT_BAKED)->getID());
	destroy_texture(avatar->getTE(TEX_HAIR_BAKED)->getID());
	LLAvatarPropertiesProcessor::getInstance()->sendAvatarTexturesRequest(avatar->getID());
}

class LLAvatarTexRefresh : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (avatar)
		{
			avatar_tex_refresh(avatar);
		}

		return true;
	}
};
// </FS:Zi> Texture Refresh

class LLObjectReportAbuse : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (objectp)
		{
			LLFloaterReporter::showFromObject(objectp->getID());
		}
		return true;
	}
};

// Enabled it you clicked an object
class LLObjectEnableReportAbuse : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLSelectMgr::getInstance()->getSelection()->getObjectCount() != 0;
		return new_value;
	}
};


void handle_object_touch()
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (!object) return;

	LLPickInfo pick = LLToolPie::getInstance()->getPick();

// [RLVa:KB] - Checked: 2010-04-11 (RLVa-1.2.0e) | Modified: RLVa-1.1.0l
	// NOTE: fallback code since we really shouldn't be getting an active selection if we can't touch this
	if ( (RlvActions::isRlvEnabled()) && (!RlvActions::canTouch(object, pick.mObjectOffset)) )
	{
		RLV_ASSERT(false);
		return;
	}
// [/RLVa:KB]

	// *NOTE: Hope the packets arrive safely and in order or else
	// there will be some problems.
	// *TODO: Just fix this bad assumption.
	send_ObjectGrab_message(object, pick, LLVector3::zero);
	send_ObjectDeGrab_message(object, pick);
}


static void init_default_item_label(const std::string& item_name)
{
	boost::unordered_map<std::string, LLStringExplicit>::iterator it = sDefaultItemLabels.find(item_name);
	if (it == sDefaultItemLabels.end())
	{
		// *NOTE: This will not work for items of type LLMenuItemCheckGL because they return boolean value
		//       (doesn't seem to matter much ATM).
		LLStringExplicit default_label = gMenuHolder->childGetValue(item_name).asString();
		if (!default_label.empty())
		{
			sDefaultItemLabels.insert(std::pair<std::string, LLStringExplicit>(item_name, default_label));
		}
	}
}

static LLStringExplicit get_default_item_label(const std::string& item_name)
{
	LLStringExplicit res("");
	boost::unordered_map<std::string, LLStringExplicit>::iterator it = sDefaultItemLabels.find(item_name);
	if (it != sDefaultItemLabels.end())
	{
		res = it->second;
	}

	return res;
}


bool enable_object_touch(LLUICtrl* ctrl)
{
	bool new_value = false;
	LLViewerObject* obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (obj)
	{
		LLViewerObject* parent = (LLViewerObject*)obj->getParent();
		new_value = obj->flagHandleTouch() || (parent && parent->flagHandleTouch());
	}

// [RLVa:KB] - Checked: 2010-11-12 (RLVa-1.2.1g) | Added: RLVa-1.2.1g
	if ( (RlvActions::isRlvEnabled()) && (new_value) )
	{
		// RELEASE-RLVa: [RLVa-1.2.1] Make sure this stays in sync with handle_object_touch()
		new_value = RlvActions::canTouch(obj, LLToolPie::getInstance()->getPick().mObjectOffset);
	}
// [/RLVa:KB]

	std::string item_name = ctrl->getName();
	init_default_item_label(item_name);

	// Update label based on the node touch name if available.
	LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
	if (node && node->mValid && !node->mTouchName.empty())
	{
		gMenuHolder->childSetValue(item_name, node->mTouchName);
	}
	else
	{
		gMenuHolder->childSetValue(item_name, get_default_item_label(item_name));
	}

	return new_value;
};

//void label_touch(std::string& label, void*)
//{
//	LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
//	if (node && node->mValid && !node->mTouchName.empty())
//	{
//		label.assign(node->mTouchName);
//	}
//	else
//	{
//		label.assign("Touch");
//	}
//}

void handle_object_open()
{
// [RLVa:KB] - Checked: 2010-04-11 (RLVa-1.2.0e) | Added: RLVa-1.2.0e
	if (enable_object_open())
		LLFloaterReg::showInstance("openobject");
// [/RLVa:KB]
//	LLFloaterReg::showInstance("openobject");
}

bool enable_object_open()
{
	// Look for contents in root object, which is all the LLFloaterOpenObject
	// understands.
	LLViewerObject* obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (!obj) return false;

	LLViewerObject* root = obj->getRootEdit();
	if (!root) return false;

	return root->allowOpen();
}


class LLViewJoystickFlycam : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_toggle_flycam();
		return true;
	}
};

class LLViewCheckJoystickFlycam : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLViewerJoystick::getInstance()->getOverrideCamera();
		return new_value;
	}
};

void handle_toggle_flycam()
{
	LLViewerJoystick::getInstance()->toggleFlycam();
}

class LLObjectBuild : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgentCamera.getFocusOnAvatar() && !LLToolMgr::getInstance()->inEdit() && gSavedSettings.getBOOL("EditCameraMovement") )
		{
			// zoom in if we're looking at the avatar
			gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());
			gAgentCamera.cameraZoomIn(0.666f);
			gAgentCamera.cameraOrbitOver( 30.f * DEG_TO_RAD );
			gViewerWindow->moveCursorToCenter();
		}
		else if ( gSavedSettings.getBOOL("EditCameraMovement") )
		{
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());
			gViewerWindow->moveCursorToCenter();
		}

		LLToolMgr::getInstance()->setCurrentToolset(gBasicToolset);
		LLToolMgr::getInstance()->getCurrentToolset()->selectTool( LLToolCompCreate::getInstance() );

		// Could be first use
		//LLFirstUse::useBuild();
		return true;
	}
};


void handle_object_edit()
{
	LLViewerParcelMgr::getInstance()->deselectLand();

	if (gAgentCamera.getFocusOnAvatar() && !LLToolMgr::getInstance()->inEdit())
	{
		LLFloaterTools::sPreviousFocusOnAvatar = true;
		LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();

		if (selection->getSelectType() == SELECT_TYPE_HUD || !gSavedSettings.getBOOL("EditCameraMovement"))
		{
			// always freeze camera in space, even if camera doesn't move
			// so, for example, follow cam scripts can't affect you when in build mode
			gAgentCamera.setFocusGlobal(gAgentCamera.calcFocusPositionTargetGlobal(), LLUUID::null);
			gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);
		}
		else
		{
			gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);
			LLViewerObject* selected_objectp = selection->getFirstRootObject();
			if (selected_objectp)
			{
			  // zoom in on object center instead of where we clicked, as we need to see the manipulator handles
			  gAgentCamera.setFocusGlobal(selected_objectp->getPositionGlobal(), selected_objectp->getID());
			  gAgentCamera.cameraZoomIn(0.666f);
			  gAgentCamera.cameraOrbitOver( 30.f * DEG_TO_RAD );
			  gViewerWindow->moveCursorToCenter();
			}
		}
	}
	
	LLFloaterReg::showInstance("build");
	
	LLToolMgr::getInstance()->setCurrentToolset(gBasicToolset);
	gFloaterTools->setEditTool( LLToolCompTranslate::getInstance() );
	
	LLViewerJoystick::getInstance()->moveObjects(true);
	LLViewerJoystick::getInstance()->setNeedsReset(true);
	
	// Could be first use
	//LLFirstUse::useBuild();
	return;
}

// [SL:KB] - Patch: Inventory-AttachmentEdit - Checked: 2010-08-25 (Catznip-2.2.0a) | Added: Catznip-2.1.2a
void handle_attachment_edit(const LLUUID& idItem)
{
	const LLInventoryItem* pItem = gInventory.getItem(idItem);
	if ( (!isAgentAvatarValid()) || (!pItem) )
		return;

	LLViewerObject* pAttachObj = gAgentAvatarp->getWornAttachment(pItem->getLinkedUUID());
	if (!pAttachObj)
		return;

	LLSelectMgr::getInstance()->deselectAll();
	LLSelectMgr::getInstance()->selectObjectAndFamily(pAttachObj);

	handle_object_edit();
}
// [/SL:KB]

void handle_object_inspect()
{
	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
	LLViewerObject* selected_objectp = selection->getFirstRootObject();
	if (selected_objectp)
	{
		LLSD key;
		key["task"] = "task";
		LLFloaterSidePanelContainer::showPanel("inventory", key);
	}
	
	/*
	// Old floater properties
	LLFloaterReg::showInstance("inspect", LLSD());
	*/
}

//---------------------------------------------------------------------------
// Land pie menu
//---------------------------------------------------------------------------
class LLLandBuild : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerParcelMgr::getInstance()->deselectLand();

		if (gAgentCamera.getFocusOnAvatar() && !LLToolMgr::getInstance()->inEdit() && gSavedSettings.getBOOL("EditCameraMovement") )
		{
			// zoom in if we're looking at the avatar
			gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());
			gAgentCamera.cameraZoomIn(0.666f);
			gAgentCamera.cameraOrbitOver( 30.f * DEG_TO_RAD );
			gViewerWindow->moveCursorToCenter();
		}
		else if ( gSavedSettings.getBOOL("EditCameraMovement")  )
		{
			// otherwise just move focus
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());
			gViewerWindow->moveCursorToCenter();
		}


		LLToolMgr::getInstance()->setCurrentToolset(gBasicToolset);
		LLToolMgr::getInstance()->getCurrentToolset()->selectTool( LLToolCompCreate::getInstance() );

		// Could be first use
		//LLFirstUse::useBuild();
		return true;
	}
};

class LLLandBuyPass : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLPanelLandGeneral::onClickBuyPass((void *)FALSE);
		return true;
	}
};

class LLLandEnableBuyPass : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLPanelLandGeneral::enableBuyPass(NULL);
		return new_value;
	}
};

// BUG: Should really check if CLICK POINT is in a parcel where you can build.
BOOL enable_land_build(void*)
{
	if (gAgent.isGodlike()) return TRUE;
	if (gAgent.inPrelude()) return FALSE;

	BOOL can_build = FALSE;
	LLParcel* agent_parcel = LLViewerParcelMgr::getInstance()->getAgentParcel();
	if (agent_parcel)
	{
		can_build = agent_parcel->getAllowModify();
	}
	return can_build;
}

// BUG: Should really check if OBJECT is in a parcel where you can build.
BOOL enable_object_build(void*)
{
	if (gAgent.isGodlike()) return TRUE;
	if (gAgent.inPrelude()) return FALSE;

	BOOL can_build = FALSE;
	LLParcel* agent_parcel = LLViewerParcelMgr::getInstance()->getAgentParcel();
	if (agent_parcel)
	{
		can_build = agent_parcel->getAllowModify();
	}
	return can_build;
}

bool enable_object_edit()
{
	if (!isAgentAvatarValid()) return false;
	
	// *HACK:  The new "prelude" Help Islands have a build sandbox area,
	// so users need the Edit and Create pie menu options when they are
	// there.  Eventually this needs to be replaced with code that only 
	// lets you edit objects if you have permission to do so (edit perms,
	// group edit, god).  See also lltoolbar.cpp.  JC
	bool enable = false;
	if (gAgent.inPrelude())
	{
		enable = LLViewerParcelMgr::getInstance()->allowAgentBuild()
			|| LLSelectMgr::getInstance()->getSelection()->isAttachment();
	} 
	else if (LLSelectMgr::getInstance()->selectGetAllValidAndObjectsFound())
	{
//		enable = true;
// [RLVa:KB] - Checked: 2010-11-29 (RLVa-1.3.0c) | Modified: RLVa-1.3.0c
		bool fRlvCanEdit = (!gRlvHandler.hasBehaviour(RLV_BHVR_EDIT)) && (!gRlvHandler.hasBehaviour(RLV_BHVR_EDITOBJ));
		if (!fRlvCanEdit)
		{
			LLObjectSelectionHandle hSel = LLSelectMgr::getInstance()->getSelection();
			RlvSelectIsEditable f;
			fRlvCanEdit = (hSel.notNull()) && ((hSel->getFirstRootNode(&f, TRUE)) == NULL);
		}
		enable = fRlvCanEdit;
// [/RLVa:KB]
	}

	return enable;
}

bool enable_mute_particle()
{
	const LLPickInfo& pick = LLToolPie::getInstance()->getPick();

	return pick.mParticleOwnerID != LLUUID::null && pick.mParticleOwnerID != gAgent.getID();
}

// mutually exclusive - show either edit option or build in menu
bool enable_object_build()
{
	return !enable_object_edit();
}

bool enable_object_select_in_pathfinding_linksets()
{
	return LLPathfindingManager::getInstance()->isPathfindingEnabledForCurrentRegion() && LLSelectMgr::getInstance()->selectGetEditableLinksets();
}

bool visible_object_select_in_pathfinding_linksets()
{
	return LLPathfindingManager::getInstance()->isPathfindingEnabledForCurrentRegion();
}

bool enable_object_select_in_pathfinding_characters()
{
	return LLPathfindingManager::getInstance()->isPathfindingEnabledForCurrentRegion() &&  LLSelectMgr::getInstance()->selectGetViewableCharacters();
}

class LLSelfRemoveAllAttachments : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLAppearanceMgr::instance().removeAllAttachmentsFromAvatar();
		return true;
	}
};

class LLSelfEnableRemoveAllAttachments : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = false;
		if (isAgentAvatarValid())
		{
			for (LLVOAvatar::attachment_map_t::iterator iter = gAgentAvatarp->mAttachmentPoints.begin(); 
				 iter != gAgentAvatarp->mAttachmentPoints.end(); )
			{
				LLVOAvatar::attachment_map_t::iterator curiter = iter++;
				LLViewerJointAttachment* attachment = curiter->second;
//				if (attachment->getNumObjects() > 0)
// [RLVa:KB] - Checked: 2010-03-04 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
				if ( (attachment->getNumObjects() > 0) && ((!rlv_handler_t::isEnabled()) || (gRlvAttachmentLocks.canDetach(attachment))) )
// [/RLVa:KB]
				{
					new_value = true;
					break;
				}
			}
		}
		return new_value;
	}
};

BOOL enable_has_attachments(void*)
{

	return FALSE;
}

//---------------------------------------------------------------------------
// Avatar pie menu
//---------------------------------------------------------------------------
//void handle_follow(void *userdata)
//{
//	// follow a given avatar by ID
//	LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
//	if (objectp)
//	{
//		gAgent.startFollowPilot(objectp->getID());
//	}
//}

bool enable_object_mute()
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (!object) return false;

	LLVOAvatar* avatar = find_avatar_from_object(object); 
	if (avatar)
	{
		// It's an avatar
		LLNameValue *lastname = avatar->getNVPair("LastName");
		bool is_linden =
			lastname && !LLStringUtil::compareStrings(lastname->getString(), "Linden");
		bool is_self = avatar->isSelf();
//		return !is_linden && !is_self;
// [RLVa:KB] - Checked: RLVa-1.2.1
//		return !is_linden && !is_self && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID()));
// [/RLVa:KB]

		// <FS:Zi> Make enable/disable of block/unblock menu items work for avatars
		if(is_linden || is_self)
			return false;

		if (!RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID()))
			return false;

		LLNameValue *firstname = avatar->getNVPair("FirstName");

		std::string name;
		if (firstname && lastname)
		{
			name = LLCacheName::buildFullName(
				firstname->getString(), lastname->getString());
		}

		LLMute mute(avatar->getID(),name,LLMute::AGENT);
		return !LLMuteList::getInstance()->isMuted(mute.mID);
		// </FS:Zi>
	}
	else
	{
		// Just a regular object
		return LLSelectMgr::getInstance()->getSelection()->contains( object, SELECT_ALL_TES ) &&
			   !LLMuteList::getInstance()->isMuted(object->getID());
	}
}

bool enable_object_unmute()
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (!object) return false;

	LLVOAvatar* avatar = find_avatar_from_object(object); 
	if (avatar)
	{
		// It's an avatar
		LLNameValue *lastname = avatar->getNVPair("LastName");
		bool is_linden =
			lastname && !LLStringUtil::compareStrings(lastname->getString(), "Linden");
		bool is_self = avatar->isSelf();
		// <FS:Zi> Make enable/disable of block/unblock menu items work for avatars
		// return !is_linden && !is_self;
		if(is_linden || is_self)
			return false;

		LLNameValue *firstname = avatar->getNVPair("FirstName");
		std::string name;
		if (firstname && lastname)
		{
			name = LLCacheName::buildFullName(
				firstname->getString(), lastname->getString());
		}

		LLMute mute(avatar->getID(),name,LLMute::AGENT);
		return LLMuteList::getInstance()->isMuted(mute.mID);
		// </FS:Zi>
	}
	else
	{
		// Just a regular object
		return LLSelectMgr::getInstance()->getSelection()->contains( object, SELECT_ALL_TES ) &&
			   LLMuteList::getInstance()->isMuted(object->getID());;
	}
}

// <FS:Ansariel> Avatar render more check for pie menu
bool check_avatar_render_mode(U32 mode)
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (!object) return false;

	LLVOAvatar* avatar = find_avatar_from_object(object); 
	if (!avatar) return false;
		
	switch (mode) 
	{
		case 0:
// [RLVa:KB] - Checked: RLVa-2.2 (@setcam_avdist)
				return FSAvatarRenderPersistence::instance().getAvatarRenderSettings(avatar->getID()) == LLVOAvatar::AV_RENDER_NORMALLY;
// [/RLVa:KB]
//				return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_RENDER_NORMALLY);
		case 1:
// [RLVa:KB] - Checked: RLVa-2.2 (@setcam_avdist)
				return FSAvatarRenderPersistence::instance().getAvatarRenderSettings(avatar->getID()) == LLVOAvatar::AV_DO_NOT_RENDER;
// [/RLVa:KB]
//				return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_DO_NOT_RENDER);
		case 2:
// [RLVa:KB] - Checked: RLVa-2.2 (@setcam_avdist)
				return FSAvatarRenderPersistence::instance().getAvatarRenderSettings(avatar->getID()) == LLVOAvatar::AV_ALWAYS_RENDER;
// [/RLVa:KB]
//				return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_ALWAYS_RENDER);
		default:
			return false;
	}
}
// </FS:Ansariel>

// 0 = normal, 1 = always, 2 = never
class LLAvatarCheckImpostorMode : public view_listener_t
{	
	bool handleEvent(const LLSD& userdata)
	{
		// <FS:Ansariel> Avatar render more check for pie menu
		//LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		//if (!object) return false;

		//LLVOAvatar* avatar = find_avatar_from_object(object); 
		//if (!avatar) return false;
		//
		//U32 mode = userdata.asInteger();
		//switch (mode) 
		//{
		//	case 0:
		//		return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_RENDER_NORMALLY);
		//	case 1:
		//		return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_DO_NOT_RENDER);
		//	case 2:
		//		return (avatar->getVisualMuteSettings() == LLVOAvatar::AV_ALWAYS_RENDER);
		//	default:
		//		return false;
		//}
		return check_avatar_render_mode(userdata.asInteger());
		// </FS:Ansariel>
	}	// handleEvent()
};

// 0 = normal, 1 = always, 2 = never
class LLAvatarSetImpostorMode : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (!object) return false;

		LLVOAvatar* avatar = find_avatar_from_object(object); 
		if (!avatar) return false;
		
		U32 mode = userdata.asInteger();
		switch (mode) 
		{
			case 0:
				avatar->setVisualMuteSettings(LLVOAvatar::AV_RENDER_NORMALLY);
				break;
			case 1:
				avatar->setVisualMuteSettings(LLVOAvatar::AV_DO_NOT_RENDER);
				break;
			case 2:
				avatar->setVisualMuteSettings(LLVOAvatar::AV_ALWAYS_RENDER);
				break;
			default:
				return false;
		}

		LLVOAvatar::cullAvatarsByPixelArea();
		return true;
	}	// handleEvent()
};


class LLObjectMute : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (!object) return true;
		
		LLUUID id;
		std::string name;
		LLMute::EType type;
		LLVOAvatar* avatar = find_avatar_from_object(object); 
		if (avatar)
		{
			id = avatar->getID();
// [RLVa:KB] - Checked: RLVa-1.0.0
			if (!RlvActions::canShowName(RlvActions::SNC_DEFAULT, id))
				return true;
// [/RLVa:KB]

			avatar->mNeedsImpostorUpdate = TRUE;


			LLNameValue *firstname = avatar->getNVPair("FirstName");
			LLNameValue *lastname = avatar->getNVPair("LastName");
			if (firstname && lastname)
			{
				name = LLCacheName::buildFullName(
					firstname->getString(), lastname->getString());
			}
			
			type = LLMute::AGENT;
		}
		else
		{
			// it's an object
			id = object->getID();

			LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
			if (node)
			{
				name = node->mName;
			}
			
			type = LLMute::OBJECT;
		}
		
		LLMute mute(id, name, type);
		if (LLMuteList::getInstance()->isMuted(mute.mID))
		{
			LLMuteList::getInstance()->remove(mute);
		}
		else
		{
			LLMuteList::getInstance()->add(mute);
			LLPanelBlockedList::showPanelAndSelect(mute.mID);
		}
		
		return true;
	}
};

bool handle_go_to()
{
	// try simulator autopilot
	std::vector<std::string> strings;
	std::string val;
	LLVector3d pos = LLToolPie::getInstance()->getPick().mPosGlobal;
	val = llformat("%g", pos.mdV[VX]);
	strings.push_back(val);
	val = llformat("%g", pos.mdV[VY]);
	strings.push_back(val);
	val = llformat("%g", pos.mdV[VZ]);
	strings.push_back(val);
	send_generic_message("autopilot", strings);

	LLViewerParcelMgr::getInstance()->deselectLand();

	if (isAgentAvatarValid() && !gSavedSettings.getBOOL("AutoPilotLocksCamera"))
	{
		gAgentCamera.setFocusGlobal(gAgentCamera.getFocusTargetGlobal(), gAgentAvatarp->getID());
	}
	else 
	{
		// Snap camera back to behind avatar
		gAgentCamera.setFocusOnAvatar(TRUE, ANIMATE);
	}

	// Could be first use
	//LLFirstUse::useGoTo();
	return true;
}

class LLGoToObject : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return handle_go_to();
	}
};

class LLAvatarReportAbuse : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
		if(avatar)
		{
			LLFloaterReporter::showFromObject(avatar->getID());
		}
		return true;
	}
};


//---------------------------------------------------------------------------
// Parcel freeze, eject, etc.
//---------------------------------------------------------------------------
//bool callback_freeze(const LLSD& notification, const LLSD& response)
//{
//	LLUUID avatar_id = notification["payload"]["avatar_id"].asUUID();
//	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
//
//	if (0 == option || 1 == option)
//	{
//		U32 flags = 0x0;
//		if (1 == option)
//		{
//			// unfreeze
//			flags |= 0x1;
//		}
//
//		LLMessageSystem* msg = gMessageSystem;
//		LLViewerObject* avatar = gObjectList.findObject(avatar_id);
//
//		if (avatar)
//		{
//			msg->newMessage("FreezeUser");
//			msg->nextBlock("AgentData");
//			msg->addUUID("AgentID", gAgent.getID());
//			msg->addUUID("SessionID", gAgent.getSessionID());
//			msg->nextBlock("Data");
//			msg->addUUID("TargetID", avatar_id );
//			msg->addU32("Flags", flags );
//			msg->sendReliable( avatar->getRegion()->getHost() );
//		}
//	}
//	return false;
//}


void handle_avatar_freeze(const LLSD& avatar_id)
{
// [SL:KB] - Patch: UI-AvatarNearbyActions | Checked: 2011-05-13 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
	// Use avatar_id if available, otherwise default to right-click avatar
	LLUUID idAgent = avatar_id.asUUID();
	if (idAgent.isNull())
	{
		/*const*/ LLVOAvatar* pAvatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (pAvatar)
			idAgent = pAvatar->getID();
	}
	if (idAgent.notNull())
	{
		LLAvatarActions::landFreeze(idAgent);
	}
// [/SL:KB]
//		// Use avatar_id if available, otherwise default to right-click avatar
//		LLVOAvatar* avatar = NULL;
//		if (avatar_id.asUUID().notNull())
//		{
//			avatar = find_avatar_from_object(avatar_id.asUUID());
//		}
//		else
//		{
//			avatar = find_avatar_from_object(
//				LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
//		}
//
//		if( avatar )
//		{
//			std::string fullname = avatar->getFullname();
//			LLSD payload;
//			payload["avatar_id"] = avatar->getID();
//
//			if (!fullname.empty())
//			{
//				LLSD args;
//				args["AVATAR_NAME"] = fullname;
// [RLVa:KB] - Checked: RLVa-1.0.0
//				args["AVATAR_NAME"] = (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) ? fullname : RlvStrings::getAnonym(fullname);
// [/RLVa:KB]
//				LLNotificationsUtil::add("FreezeAvatarFullname",
//							args,
//							payload,
//							callback_freeze);
//			}
//			else
//			{
//				LLNotificationsUtil::add("FreezeAvatar",
//							LLSD(),
//							payload,
//							callback_freeze);
//			}
//		}
}

class LLAvatarVisibleDebug : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return gAgent.isGodlike();
	}
};

class LLAvatarDebug : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
		if( avatar )
		{
			if (avatar->isSelf())
			{
				((LLVOAvatarSelf *)avatar)->dumpLocalTextures();
			}
			LL_INFOS() << "Dumping temporary asset data to simulator logs for avatar " << avatar->getID() << LL_ENDL;
			// <FS:Ansariel> Disable message - spawns error "generic request failed"
			//std::vector<std::string> strings;
			//strings.push_back(avatar->getID().asString());
			//LLUUID invoice;
			//send_generic_message("dumptempassetdata", strings, invoice);
			// </FS:Ansariel>
			LLFloaterReg::showInstance( "avatar_textures", LLSD(avatar->getID()) );
		}
		return true;
	}
};

//bool callback_eject(const LLSD& notification, const LLSD& response)
//{
//	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
//	if (2 == option)
//	{
//		// Cancel button.
//		return false;
//	}
//	LLUUID avatar_id = notification["payload"]["avatar_id"].asUUID();
//	bool ban_enabled = notification["payload"]["ban_enabled"].asBoolean();
//
//	if (0 == option)
//	{
//		// Eject button
//		LLMessageSystem* msg = gMessageSystem;
//		LLViewerObject* avatar = gObjectList.findObject(avatar_id);
//
//		if (avatar)
//		{
//			U32 flags = 0x0;
//			msg->newMessage("EjectUser");
//			msg->nextBlock("AgentData");
//			msg->addUUID("AgentID", gAgent.getID() );
//			msg->addUUID("SessionID", gAgent.getSessionID() );
//			msg->nextBlock("Data");
//			msg->addUUID("TargetID", avatar_id );
//			msg->addU32("Flags", flags );
//			msg->sendReliable( avatar->getRegion()->getHost() );
//		}
//	}
//	else if (ban_enabled)
//	{
//		// This is tricky. It is similar to say if it is not an 'Eject' button,
//		// and it is also not an 'Cancle' button, and ban_enabled==ture, 
//		// it should be the 'Eject and Ban' button.
//		LLMessageSystem* msg = gMessageSystem;
//		LLViewerObject* avatar = gObjectList.findObject(avatar_id);
//
//		if (avatar)
//		{
//			U32 flags = 0x1;
//			msg->newMessage("EjectUser");
//			msg->nextBlock("AgentData");
//			msg->addUUID("AgentID", gAgent.getID() );
//			msg->addUUID("SessionID", gAgent.getSessionID() );
//			msg->nextBlock("Data");
//			msg->addUUID("TargetID", avatar_id );
//			msg->addU32("Flags", flags );
//			msg->sendReliable( avatar->getRegion()->getHost() );
//		}
//	}
//	return false;
//}

void handle_avatar_eject(const LLSD& avatar_id)
{
// [SL:KB] - Patch: UI-AvatarNearbyActions | Checked: 2011-05-13 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
	// Use avatar_id if available, otherwise default to right-click avatar
	LLUUID idAgent = avatar_id.asUUID();
	if (idAgent.isNull())
	{
		/*const*/ LLVOAvatar* pAvatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (pAvatar)
			idAgent = pAvatar->getID();
	}
	if (idAgent.notNull())
	{
		LLAvatarActions::landEject(idAgent);
	}
// [/SL:KB]
//		// Use avatar_id if available, otherwise default to right-click avatar
//		LLVOAvatar* avatar = NULL;
//		if (avatar_id.asUUID().notNull())
//		{
//			avatar = find_avatar_from_object(avatar_id.asUUID());
//		}
//		else
//		{
//			avatar = find_avatar_from_object(
//				LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
//		}
//
//		if( avatar )
//		{
//			LLSD payload;
//			payload["avatar_id"] = avatar->getID();
//			std::string fullname = avatar->getFullname();
//
//			const LLVector3d& pos = avatar->getPositionGlobal();
//			LLParcel* parcel = LLViewerParcelMgr::getInstance()->selectParcelAt(pos)->getParcel();
//			
//			if (LLViewerParcelMgr::getInstance()->isParcelOwnedByAgent(parcel,GP_LAND_MANAGE_BANNED))
//			{
//                payload["ban_enabled"] = true;
//				if (!fullname.empty())
//				{
//    				LLSD args;
//					args["AVATAR_NAME"] = fullname;
// [RLVa:KB] - Checked: RLVa-1.0.0
//					args["AVATAR_NAME"] = (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) ? fullname : RlvStrings::getAnonym(fullname);
// [/RLVa:KB]
//    				LLNotificationsUtil::add("EjectAvatarFullname",
//    							args,
//    							payload,
//    							callback_eject);
//				}
//				else
//				{
//    				LLNotificationsUtil::add("EjectAvatarFullname",
//    							LLSD(),
//    							payload,
//    							callback_eject);
//				}
//			}
//			else
//			{
//                payload["ban_enabled"] = false;
//				if (!fullname.empty())
//				{
//    				LLSD args;
//					args["AVATAR_NAME"] = fullname;
// [RLVa:KB] - Checked: RLVa-1.0.0
//					args["AVATAR_NAME"] = (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) ? fullname : RlvStrings::getAnonym(fullname);
// [/RLVa:KB]
//    				LLNotificationsUtil::add("EjectAvatarFullnameNoBan",
//    							args,
//    							payload,
//    							callback_eject);
//				}
//				else
//				{
//    				LLNotificationsUtil::add("EjectAvatarNoBan",
//    							LLSD(),
//    							payload,
//    							callback_eject);
//				}
//			}
//		}
}

bool my_profile_visible()
{
	LLFloater* floaterp = LLAvatarActions::getProfileFloater(gAgentID);
	return floaterp && floaterp->isInVisibleChain();
}

bool enable_freeze_eject(const LLSD& avatar_id)
{
// [SL:KB] - Patch: UI-AvatarNearbyActions | Checked: 2011-05-13 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
	// Use avatar_id if available, otherwise default to right-click avatar
	LLUUID idAgent = avatar_id.asUUID();
	if (idAgent.isNull())
	{
		/*const*/ LLVOAvatar* pAvatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (pAvatar)
			idAgent = pAvatar->getID();
	}
	return (idAgent.notNull()) ? LLAvatarActions::canLandFreezeOrEject(idAgent) : false;
// [/SL:KB]
//	// Use avatar_id if available, otherwise default to right-click avatar
//	LLVOAvatar* avatar = NULL;
//	if (avatar_id.asUUID().notNull())
//	{
//		avatar = find_avatar_from_object(avatar_id.asUUID());
//	}
//	else
//	{
//		avatar = find_avatar_from_object(
//			LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
//	}
//	if (!avatar) return false;
//
//	// Gods can always freeze
//	if (gAgent.isGodlike()) return true;
//
//	// Estate owners / managers can freeze
//	// Parcel owners can also freeze
//	const LLVector3& pos = avatar->getPositionRegion();
//	const LLVector3d& pos_global = avatar->getPositionGlobal();
//	LLParcel* parcel = LLViewerParcelMgr::getInstance()->selectParcelAt(pos_global)->getParcel();
//	LLViewerRegion* region = avatar->getRegion();
//	if (!region) return false;
//				
//	bool new_value = region->isOwnedSelf(pos);
//	if (!new_value || region->isOwnedGroup(pos))
//	{
//		new_value = LLViewerParcelMgr::getInstance()->isParcelOwnedByAgent(parcel,GP_LAND_ADMIN);
//	}
//	return new_value;
}

// <FS:Ansariel> FIRE-13515: Re-add give calling card
class LLAvatarGiveCard : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LL_INFOS("LLAvatarGiveCard") << "handle_give_card()" << LL_ENDL;
		LLViewerObject* dest = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
// [RLVa:KB] - Checked: 2010-06-04 (RLVa-1.2.0d) | Modified: RLVa-1.2.0d | OK
		//if(dest && dest->isAvatar())
		if ( (dest && dest->isAvatar()) && (!gRlvHandler.hasBehaviour(RLV_BHVR_SHOWNAMES)) )
// [/RLVa:KB]
		{
			bool found_name = false;
			LLSD args;
			LLSD old_args;
			LLNameValue* nvfirst = dest->getNVPair("FirstName");
			LLNameValue* nvlast = dest->getNVPair("LastName");
			if(nvfirst && nvlast)
			{
				std::string full_name = gCacheName->buildFullName(nvfirst->getString(), nvlast->getString());
				args["NAME"] = full_name;
				old_args["NAME"] = full_name;
				found_name = true;
			}
			LLViewerRegion* region = dest->getRegion();
			LLHost dest_host;
			if(region)
			{
				dest_host = region->getHost();
			}
			if(found_name && dest_host.isOk())
			{
				LLMessageSystem* msg = gMessageSystem;
				msg->newMessage("OfferCallingCard");
				msg->nextBlockFast(_PREHASH_AgentData);
				msg->addUUIDFast(_PREHASH_AgentID, gAgent.getID());
				msg->addUUIDFast(_PREHASH_SessionID, gAgent.getSessionID());
				msg->nextBlockFast(_PREHASH_AgentBlock);
				msg->addUUIDFast(_PREHASH_DestID, dest->getID());
				LLUUID transaction_id;
				transaction_id.generate();
				msg->addUUIDFast(_PREHASH_TransactionID, transaction_id);
				msg->sendReliable(dest_host);
				LLNotificationsUtil::add("OfferedCard", args);
			}
			else
			{
				LLNotificationsUtil::add("CantOfferCallingCard", old_args);
			}
		}
		return true;
	}
};
// </FS:Ansariel> FIRE-13515: Re-add give calling card

bool callback_leave_group(const LLSD& notification, const LLSD& response)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (option == 0)
	{
		LLMessageSystem *msg = gMessageSystem;

		msg->newMessageFast(_PREHASH_LeaveGroupRequest);
		msg->nextBlockFast(_PREHASH_AgentData);
		msg->addUUIDFast(_PREHASH_AgentID, gAgent.getID() );
		msg->addUUIDFast(_PREHASH_SessionID, gAgent.getSessionID());
		msg->nextBlockFast(_PREHASH_GroupData);
		msg->addUUIDFast(_PREHASH_GroupID, gAgent.getGroupID() );
		gAgent.sendReliableMessage();
	}
	return false;
}

void append_aggregate(std::string& string, const LLAggregatePermissions& ag_perm, PermissionBit bit, const char* txt)
{
	LLAggregatePermissions::EValue val = ag_perm.getValue(bit);
	std::string buffer;
	switch(val)
	{
	  case LLAggregatePermissions::AP_NONE:
		buffer = llformat( "* %s None\n", txt);
		break;
	  case LLAggregatePermissions::AP_SOME:
		buffer = llformat( "* %s Some\n", txt);
		break;
	  case LLAggregatePermissions::AP_ALL:
		buffer = llformat( "* %s All\n", txt);
		break;
	  case LLAggregatePermissions::AP_EMPTY:
	  default:
		break;
	}
	string.append(buffer);
}

bool enable_buy_object()
{
    // In order to buy, there must only be 1 purchaseable object in
    // the selection manager.
	if(LLSelectMgr::getInstance()->getSelection()->getRootObjectCount() != 1) return false;
    LLViewerObject* obj = NULL;
    LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
	if(node)
    {
        obj = node->getObject();
        if(!obj) return false;

		if( for_sale_selection(node) )
		{
			// *NOTE: Is this needed?  This checks to see if anyone owns the
			// object, dating back to when we had "public" objects owned by
			// no one.  JC
			if(obj->permAnyOwner()) return true;
		}
    }
	return false;
}

// Note: This will only work if the selected object's data has been
// received by the viewer and cached in the selection manager.
void handle_buy_object(LLSaleInfo sale_info)
{
	if(!LLSelectMgr::getInstance()->selectGetAllRootsValid())
	{
		LLNotificationsUtil::add("UnableToBuyWhileDownloading");
		return;
	}

	LLUUID owner_id;
	std::string owner_name;
	BOOL owners_identical = LLSelectMgr::getInstance()->selectGetOwner(owner_id, owner_name);
	if (!owners_identical)
	{
		LLNotificationsUtil::add("CannotBuyObjectsFromDifferentOwners");
		return;
	}

	LLPermissions perm;
	BOOL valid = LLSelectMgr::getInstance()->selectGetPermissions(perm);
	LLAggregatePermissions ag_perm;
	valid &= LLSelectMgr::getInstance()->selectGetAggregatePermissions(ag_perm);
	if(!valid || !sale_info.isForSale() || !perm.allowTransferTo(gAgent.getID()))
	{
		LLNotificationsUtil::add("ObjectNotForSale");
		return;
	}

	LLFloaterBuy::show(sale_info);
}


void handle_buy_contents(LLSaleInfo sale_info)
{
	LLFloaterBuyContents::show(sale_info);
}

void handle_region_dump_temp_asset_data(void*)
{
	LL_INFOS() << "Dumping temporary asset data to simulator logs" << LL_ENDL;
	std::vector<std::string> strings;
	LLUUID invoice;
	send_generic_message("dumptempassetdata", strings, invoice);
}

void handle_region_clear_temp_asset_data(void*)
{
	LL_INFOS() << "Clearing temporary asset data" << LL_ENDL;
	std::vector<std::string> strings;
	LLUUID invoice;
	send_generic_message("cleartempassetdata", strings, invoice);
}

void handle_region_dump_settings(void*)
{
	LLViewerRegion* regionp = gAgent.getRegion();
	if (regionp)
	{
		LL_INFOS() << "Damage:    " << (regionp->getAllowDamage() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "Landmark:  " << (regionp->getAllowLandmark() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "SetHome:   " << (regionp->getAllowSetHome() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "ResetHome: " << (regionp->getResetHomeOnTeleport() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "SunFixed:  " << (regionp->getSunFixed() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "BlockFly:  " << (regionp->getBlockFly() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "AllowP2P:  " << (regionp->getAllowDirectTeleport() ? "on" : "off") << LL_ENDL;
		LL_INFOS() << "Water:     " << (regionp->getWaterHeight()) << LL_ENDL;
	}
}

void handle_dump_group_info(void *)
{
	gAgent.dumpGroupInfo();
}

void handle_dump_capabilities_info(void *)
{
	LLViewerRegion* regionp = gAgent.getRegion();
	if (regionp)
	{
		regionp->logActiveCapabilities();
	}
}

void handle_dump_region_object_cache(void*)
{
	LLViewerRegion* regionp = gAgent.getRegion();
	if (regionp)
	{
		regionp->dumpCache();
	}
}

void handle_dump_focus()
{
	LLUICtrl *ctrl = dynamic_cast<LLUICtrl*>(gFocusMgr.getKeyboardFocus());

	LL_INFOS() << "Keyboard focus " << (ctrl ? ctrl->getName() : "(none)") << LL_ENDL;
}

class LLSelfStandUp : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gAgent.standUp();
		return true;
	}
};

bool enable_standup_self()
{
// [RLVa:KB] - Checked: 2010-04-01 (RLVa-1.2.0c) | Modified: RLVa-1.0.0g
	return isAgentAvatarValid() && gAgentAvatarp->isSitting() && RlvActions::canStand();
// [/RLVa:KB]
//	return isAgentAvatarValid() && gAgentAvatarp->isSitting();
}

class LLSelfSitDown : public view_listener_t
    {
        bool handleEvent(const LLSD& userdata)
        {
            gAgent.sitDown();
            return true;
        }
    };



bool show_sitdown_self()
{
	return isAgentAvatarValid() && !gAgentAvatarp->isSitting();
}

bool enable_sitdown_self()
{
// [RLVa:KB] - Checked: 2010-08-28 (RLVa-1.2.1a) | Added: RLVa-1.2.1a
	return show_sitdown_self() && !gAgentAvatarp->isEditingAppearance() && !gAgent.getFlying() && !gRlvHandler.hasBehaviour(RLV_BHVR_SIT);
// [/RLVa:KB]
//	return show_sitdown_self() && !gAgentAvatarp->isEditingAppearance() && !gAgent.getFlying();
}

// Force sit -KC
class FSSelfForceSit : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (!gAgentAvatarp->isSitting() && !gRlvHandler.hasBehaviour(RLV_BHVR_SIT))
		{
			gAgent.sitDown();
		}
		else if (gAgentAvatarp->isSitting() && !gRlvHandler.hasBehaviour(RLV_BHVR_UNSIT))
		{
			gAgent.standUp();
		}

		return true;
	}
};

bool enable_forcesit_self()
{
	return isAgentAvatarValid() &&
		((!gAgentAvatarp->isSitting() && !gRlvHandler.hasBehaviour(RLV_BHVR_SIT)) || 
		(gAgentAvatarp->isSitting() && !gRlvHandler.hasBehaviour(RLV_BHVR_UNSIT)));
}

class FSSelfCheckForceSit : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (!isAgentAvatarValid())
		{
			return false;
		}

		return gAgentAvatarp->isSitting();
	}
};

// Phantom mode -KC & <FS:CR>
class FSSelfToggleMoveLock : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (LLGridManager::getInstance()->isInSecondLife())
		{
			make_ui_sound("UISndMovelockToggle");
			bool new_value = !gSavedPerAccountSettings.getBOOL("UseMoveLock");
			gSavedPerAccountSettings.setBOOL("UseMoveLock", new_value);
			if (new_value)
			{
				report_to_nearby_chat(LLTrans::getString("MovelockEnabling"));
			}
			else
			{
				report_to_nearby_chat(LLTrans::getString("MovelockDisabling"));
			}
		}
#ifdef OPENSIM
		else
		{
			gAgent.togglePhantom();
		}
#endif // OPENSIM
		//TODO: feedback to local chat
		return true;
	}
};


class FSSelfCheckMoveLock : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value(false);
		if (LLGridManager::getInstance()->isInSecondLife())
		{
			new_value = gSavedPerAccountSettings.getBOOL("UseMoveLock");
		}
#ifdef OPENSIM
		else
		{
			new_value = gAgent.getPhantom();
		}
#endif // OPENSIM
		return new_value;
	}
};

bool enable_bridge_function()
{
	return FSLSLBridge::instance().canUseBridge();
}

bool enable_move_lock()
{
#ifdef OPENSIM
	// Phantom mode always works on opensim, at least right now.
	if (LLGridManager::getInstance()->isInOpenSim())
		return true;
#endif // OPENSIM
	return enable_bridge_function();
}

bool enable_script_info()
{
	return (!LLSelectMgr::getInstance()->getSelection()->isEmpty()
			&& enable_bridge_function());
}
// </FS:CR>

// [SJ - Adding IgnorePrejump in Menu ]
class FSSelfToggleIgnorePreJump : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gSavedSettings.setBOOL("FSIgnoreFinishAnimation", !gSavedSettings.getBOOL("FSIgnoreFinishAnimation"));
		return true;
	}
};

// [SJ - Adding IgnorePrejump in Menu ]
class FSSelfCheckIgnorePreJump : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gSavedSettings.getBOOL("FSIgnoreFinishAnimation");
		return new_value;
	}
};

class LLCheckPanelPeopleTab : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
		{
			std::string panel_name = userdata.asString();

			LLPanel *panel = LLFloaterSidePanelContainer::getPanel("people", panel_name);
			if(panel && panel->isInVisibleChain())
			{
				return true;
			}
			return false;
		}
};
// Toggle one of "People" panel tabs in side tray.
class LLTogglePanelPeopleTab : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string panel_name = userdata.asString();

		LLSD param;
		param["people_panel_tab_name"] = panel_name;

		// <FS:Ansariel> Handle blocklist separately because of standalone option
		if (panel_name == "blocked_panel")
		{
			if (gSavedSettings.getBOOL("FSUseStandaloneBlocklistFloater"))
			{
				LLFloaterReg::showInstance("fs_blocklist");
			}
			else
			{
				togglePeoplePanel(panel_name, param);
			}
			return true;
		}
		// </FS:Ansariel>

		// <FS:Zi> Open groups and friends lists in communicate floater
		// <FS:Lo> Adding an option to still use v2 windows
		if(gSavedSettings.getBOOL("FSUseV2Friends") && gSavedSettings.getString("FSInternalSkinCurrent") != "Vintage")
		{
			if (   panel_name == "friends_panel"
				|| panel_name == "groups_panel"
				|| panel_name == "nearby_panel"
				|| panel_name == "blocked_panel"
				|| panel_name == "contact_sets_panel")
			{
				return togglePeoplePanel(panel_name, param);
			}
			else
			{
				return false;
			}
		}
		else
		{
			if(panel_name=="nearby_panel")
			{
				return togglePeoplePanel(panel_name,param);
			}
			else if(panel_name=="groups_panel")
			{
				if (gSavedSettings.getBOOL("ContactsTornOff"))
				{
					FSFloaterContacts* instance = FSFloaterContacts::getInstance();
					std::string activetab = instance->getChild<LLTabContainer>("friends_and_groups")->getCurrentPanel()->getName();
					if (instance->getVisible() && activetab == panel_name) 
					{
						instance->closeFloater();
					}
					else
					{
						instance->openTab("groups");
					}
				}
				else
				{
					FSFloaterContacts::getInstance()->openTab("groups");
				}
				return true;
			}
			else if(panel_name=="friends_panel")
			{
				if (gSavedSettings.getBOOL("ContactsTornOff"))
				{
					FSFloaterContacts* instance = FSFloaterContacts::getInstance();
					std::string activetab = instance->getChild<LLTabContainer>("friends_and_groups")->getCurrentPanel()->getName();
					if (instance->getVisible() && activetab == panel_name) 
					{
						instance->closeFloater();
					}
					else
					{
						instance->openTab("friends");
					}
				}
				else
				{
					FSFloaterContacts::getInstance()->openTab("friends");
				}
				return true;
			}
			else if(panel_name=="contact_sets_panel")
			{
				if (gSavedSettings.getBOOL("ContactsTornOff"))
				{
					FSFloaterContacts* instance = FSFloaterContacts::getInstance();
					std::string activetab = instance->getChild<LLTabContainer>("friends_and_groups")->getCurrentPanel()->getName();
					if (instance->getVisible() && activetab == panel_name)
					{
						instance->closeFloater();
					}
					else
					{
						instance->openTab("contact_sets");
					}
				}
				else
				{
					FSFloaterContacts::getInstance()->openTab("contact_sets");
				}
				return true;
			}
			else
			{
				return false;
			}
		}
		// </FS:Lo>
		// </FS:Zi>
	}

	static bool togglePeoplePanel(const std::string& panel_name, const LLSD& param)
	{
		LLPanel	*panel = LLFloaterSidePanelContainer::getPanel("people", panel_name);
		if(!panel)
			return false;

		if (panel->isInVisibleChain())
		{
			LLFloaterReg::hideInstance("people");
		}
		else
		{
			LLFloaterSidePanelContainer::showPanel("people", "panel_people", param) ;
		}

		return true;
	}
};

BOOL check_admin_override(void*)
{
	return gAgent.getAdminOverride();
}

void handle_admin_override_toggle(void*)
{
	gAgent.setAdminOverride(!gAgent.getAdminOverride());

	// The above may have affected which debug menus are visible
	show_debug_menus();
}

void handle_visual_leak_detector_toggle(void*)
{
	static bool vld_enabled = false;

	if ( vld_enabled )
	{
#ifdef INCLUDE_VLD
		// only works for debug builds (hard coded into vld.h)
#if defined(_DEBUG) || defined(VLD_FORCE_ENABLE)
		// start with Visual Leak Detector turned off
		VLDDisable();
#endif // _DEBUG
#endif // INCLUDE_VLD
		vld_enabled = false;
	}
	else
	{
#ifdef INCLUDE_VLD
		// only works for debug builds (hard coded into vld.h)
#if defined(_DEBUG) || defined(VLD_FORCE_ENABLE)
		// start with Visual Leak Detector turned off
		VLDEnable();
#endif // _DEBUG
#endif // INCLUDE_VLD

		vld_enabled = true;
	};
}

void handle_god_mode(void*)
{
	gAgent.requestEnterGodMode();
}

void handle_leave_god_mode(void*)
{
	gAgent.requestLeaveGodMode();
}

void set_god_level(U8 god_level)
{
	U8 old_god_level = gAgent.getGodLevel();
	gAgent.setGodLevel( god_level );
	LLViewerParcelMgr::getInstance()->notifyObservers();

	// God mode changes region visibility
	LLWorldMap::getInstance()->reloadItems(true);

	// inventory in items may change in god mode
	gObjectList.dirtyAllObjectInventory();

        if(gViewerWindow)
        {
            gViewerWindow->setMenuBackgroundColor(god_level > GOD_NOT,
            !LLGridManager::getInstance()->isInSLBeta());
        }
    
        LLSD args;
	if(god_level > GOD_NOT)
	{
		args["LEVEL"] = llformat("%d",(S32)god_level);
		LLNotificationsUtil::add("EnteringGodMode", args);
	}
	else
	{
		args["LEVEL"] = llformat("%d",(S32)old_god_level);
		LLNotificationsUtil::add("LeavingGodMode", args);
	}

	// changing god-level can affect which menus we see
	show_debug_menus();

	// changing god-level can invalidate search results
	LLFloaterSearch *search = dynamic_cast<LLFloaterSearch*>(LLFloaterReg::getInstance("search"));
	if (search)
	{
		search->godLevelChanged(god_level);
	}
}

#ifdef TOGGLE_HACKED_GODLIKE_VIEWER
void handle_toggle_hacked_godmode(void*)
{
	gHackGodmode = !gHackGodmode;
	set_god_level(gHackGodmode ? GOD_MAINTENANCE : GOD_NOT);
}

BOOL check_toggle_hacked_godmode(void*)
{
	return gHackGodmode;
}

bool enable_toggle_hacked_godmode(void*)
{
  return LLGridManager::getInstance()->isInSLBeta();
}
#endif

void process_grant_godlike_powers(LLMessageSystem* msg, void**)
{
	LLUUID agent_id;
	msg->getUUIDFast(_PREHASH_AgentData, _PREHASH_AgentID, agent_id);
	LLUUID session_id;
	msg->getUUIDFast(_PREHASH_AgentData, _PREHASH_SessionID, session_id);
	if((agent_id == gAgent.getID()) && (session_id == gAgent.getSessionID()))
	{
		U8 god_level;
		msg->getU8Fast(_PREHASH_GrantData, _PREHASH_GodLevel, god_level);
		set_god_level(god_level);
	}
	else
	{
		LL_WARNS() << "Grant godlike for wrong agent " << agent_id << LL_ENDL;
	}
}

/*
class LLHaveCallingcard : public LLInventoryCollectFunctor
{
public:
	LLHaveCallingcard(const LLUUID& agent_id);
	virtual ~LLHaveCallingcard() {}
	virtual bool operator()(LLInventoryCategory* cat,
							LLInventoryItem* item);
	BOOL isThere() const { return mIsThere;}
protected:
	LLUUID mID;
	BOOL mIsThere;
};

LLHaveCallingcard::LLHaveCallingcard(const LLUUID& agent_id) :
	mID(agent_id),
	mIsThere(FALSE)
{
}

bool LLHaveCallingcard::operator()(LLInventoryCategory* cat,
								   LLInventoryItem* item)
{
	if(item)
	{
		if((item->getType() == LLAssetType::AT_CALLINGCARD)
		   && (item->getCreatorUUID() == mID))
		{
			mIsThere = TRUE;
		}
	}
	return FALSE;
}
*/

BOOL is_agent_mappable(const LLUUID& agent_id)
{
	const LLRelationship* buddy_info = NULL;
	bool is_friend = LLAvatarActions::isFriend(agent_id);

	if (is_friend)
		buddy_info = LLAvatarTracker::instance().getBuddyInfo(agent_id);

	return (buddy_info &&
		buddy_info->isOnline() &&
		buddy_info->isRightGrantedFrom(LLRelationship::GRANT_MAP_LOCATION)
		);
}


// Enable a menu item when you don't have someone's card.
class LLAvatarEnableAddFriend : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
//		bool new_value = avatar && !LLAvatarActions::isFriend(avatar->getID());
// [RLVa:KB] - Checked: RLVa-1.2.0
		bool new_value = avatar && !LLAvatarActions::isFriend(avatar->getID()) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID()));
// [/RLVa:KB]
		return new_value;
	}
};

void request_friendship(const LLUUID& dest_id)
{
	LLViewerObject* dest = gObjectList.findObject(dest_id);
	if(dest && dest->isAvatar())
	{
		std::string full_name;
		LLNameValue* nvfirst = dest->getNVPair("FirstName");
		LLNameValue* nvlast = dest->getNVPair("LastName");
		if(nvfirst && nvlast)
		{
			full_name = LLCacheName::buildFullName(
				nvfirst->getString(), nvlast->getString());
		}
		if (!full_name.empty())
		{
			LLAvatarActions::requestFriendshipDialog(dest_id, full_name);
		}
		else
		{
			LLNotificationsUtil::add("CantOfferFriendship");
		}
	}
}


class LLEditEnableCustomizeAvatar : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
//		bool new_value = gAgentWearables.areWearablesLoaded();
// [RLVa:KB] - Checked: 2010-04-01 (RLVa-1.2.0c) | Modified: RLVa-1.0.0g
		bool new_value = gAgentWearables.areWearablesLoaded() && ((!rlv_handler_t::isEnabled()) || (RlvActions::canStand()));
// [/RLVa:KB]
		return new_value;
	}
};

class LLEnableEditShape : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return gAgentWearables.isWearableModifiable(LLWearableType::WT_SHAPE, 0);
	}
};

class LLEnableHoverHeight : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// <FS:Ansariel> Legacy baking avatar z-offset
		//return gAgent.getRegion() && gAgent.getRegion()->avatarHoverHeightEnabled();
		return (gAgent.getRegion() && gAgent.getRegion()->avatarHoverHeightEnabled()) || (isAgentAvatarValid() && !gAgentAvatarp->isUsingServerBakes());
		// </FS:Ansariel>
	}
};

class LLEnableEditPhysics : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		//return gAgentWearables.isWearableModifiable(LLWearableType::WT_SHAPE, 0);
		return TRUE;
	}
};

bool is_object_sittable()
{
// [RLVa:KB] - Checked: 2010-03-06 (RLVa-1.2.0c) | Added: RLVa-1.1.0j
	// RELEASE-RLVa: [SL-2.2.0] Make sure we're examining the same object that handle_sit_or_stand() will request a sit for
	if (rlv_handler_t::isEnabled())
	{
		const LLPickInfo& pick = LLToolPie::getInstance()->getPick();
		if ( (pick.mObjectID.notNull()) && (!RlvActions::canSit(pick.getObject(), pick.mObjectOffset)) )
			return false;
	}
// [/RLVa:KB]

	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();

	if (object && object->getPCode() == LL_PCODE_VOLUME)
	{
		return true;
	}
	else
	{
		return false;
	}
}


// only works on pie menu
void handle_object_sit_or_stand()
{
	LLPickInfo pick = LLToolPie::getInstance()->getPick();
	LLViewerObject *object = pick.getObject();;
	if (!object || pick.mPickType == LLPickInfo::PICK_FLORA)
	{
		return;
	}

	if (sitting_on_selection())
	{
		gAgent.standUp();
		return;
	}

	// get object selection offset 

//	if (object && object->getPCode() == LL_PCODE_VOLUME)
// [RLVa:KB] - Checked: 2010-03-06 (RLVa-1.2.0c) | Modified: RLVa-1.2.0c
	if ( (object && object->getPCode() == LL_PCODE_VOLUME) && 
		 ((!rlv_handler_t::isEnabled()) || (RlvActions::canSit(object, pick.mObjectOffset))) )
// [/RLVa:KB]
	{
// [RLVa:KB] - Checked: 2010-08-29 (RLVa-1.2.1c) | Added: RLVa-1.2.1c
		if ( (gRlvHandler.hasBehaviour(RLV_BHVR_STANDTP)) && (isAgentAvatarValid()) )
		{
			if (gAgentAvatarp->isSitting())
			{
				gAgent.standUp();
				return;
			}
			gRlvHandler.setSitSource(gAgent.getPositionGlobal());
		}
// [/RLVa:KB]

		gMessageSystem->newMessageFast(_PREHASH_AgentRequestSit);
		gMessageSystem->nextBlockFast(_PREHASH_AgentData);
		gMessageSystem->addUUIDFast(_PREHASH_AgentID, gAgent.getID());
		gMessageSystem->addUUIDFast(_PREHASH_SessionID, gAgent.getSessionID());
		gMessageSystem->nextBlockFast(_PREHASH_TargetObject);
		gMessageSystem->addUUIDFast(_PREHASH_TargetID, object->mID);
		gMessageSystem->addVector3Fast(_PREHASH_Offset, pick.mObjectOffset);

		object->getRegion()->sendReliableMessage();
	}
}

void near_sit_down_point(BOOL success, void *)
{
	if (success)
	{
		gAgent.setFlying(FALSE);
		gAgent.setControlFlags(AGENT_CONTROL_SIT_ON_GROUND);

		// Might be first sit
		//LLFirstUse::useSit();
	}
}

class LLLandSit : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
		if ( (rlv_handler_t::isEnabled()) && ((!RlvActions::canStand()) || (gRlvHandler.hasBehaviour(RLV_BHVR_SIT))) )
			return true;
// [/RLVa:KB]

		gAgent.standUp();
		LLViewerParcelMgr::getInstance()->deselectLand();

		LLVector3d posGlobal = LLToolPie::getInstance()->getPick().mPosGlobal;
		
		LLQuaternion target_rot;
		if (isAgentAvatarValid())
		{
			target_rot = gAgentAvatarp->getRotation();
		}
		else
		{
			target_rot = gAgent.getFrameAgent().getQuaternion();
		}
		gAgent.startAutoPilotGlobal(posGlobal, "Sit", &target_rot, near_sit_down_point, NULL, 0.7f);
		return true;
	}
};

//-------------------------------------------------------------------
// Help menu functions
//-------------------------------------------------------------------

//
// Major mode switching
//
void reset_view_final( BOOL proceed );

void handle_reset_view()
{
	if (gAgentCamera.cameraCustomizeAvatar())
	{
		// switching to outfit selector should automagically save any currently edited wearable
		LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "my_outfits"));
	}

	// <FS:Zi> Added optional V1 behavior so the avatar turns into camera direction after hitting ESC
	if(gSavedSettings.getBOOL("ResetViewTurnsAvatar"))
		gAgentCamera.resetView();
	// </FS:Zi>

	gAgentCamera.switchCameraPreset(CAMERA_PRESET_REAR_VIEW);
	reset_view_final( TRUE );
	LLFloaterCamera::resetCameraMode();
}

// <FS:Zi> Add reset camera angles menu
void handle_reset_camera_angles()
{
	handle_reset_view();

	// Camera focus and offset with CTRL/SHIFT + Scroll wheel
	gSavedSettings.getControl("FocusOffsetRearView")->resetToDefault();
	gSavedSettings.getControl("CameraOffsetRearView")->resetToDefault();
}
// </FS:Zi>

class LLViewResetView : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_reset_view();
		return true;
	}
};

// <FS:Zi> Add reset camera angles menu
class LLViewResetCameraAngles : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		handle_reset_camera_angles();
		return true;
	}
};
// </FS:Zi>

// Note: extra parameters allow this function to be called from dialog.
void reset_view_final( BOOL proceed ) 
{
	if( !proceed )
	{
		return;
	}

	gAgentCamera.resetView(TRUE, TRUE);
	gAgentCamera.setLookAt(LOOKAT_TARGET_CLEAR);
}

class LLViewLookAtLastChatter : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gAgentCamera.lookAtLastChat();
		return true;
	}
};

class LLViewMouselook : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (!gAgentCamera.cameraMouselook())
		{
			gAgentCamera.changeCameraToMouselook();
		}
		else
		{
			// NaCl - Rightclick-mousewheel zoom
			static LLCachedControl<LLVector3> _NACL_MLFovValues(gSavedSettings,"_NACL_MLFovValues");
			static LLCachedControl<F32> CameraAngle(gSavedSettings,"CameraAngle");
			LLVector3 vTemp=_NACL_MLFovValues;
			if(vTemp.mV[2] > 0.0f)
			{
				vTemp.mV[1]=CameraAngle;
				vTemp.mV[2]=0.0f;
				gSavedSettings.setVector3("_NACL_MLFovValues",vTemp);
				gSavedSettings.setF32("CameraAngle",vTemp.mV[0]);
			}
			// NaCl End
			gAgentCamera.changeCameraToDefault();
		}
		return true;
	}
};

class LLViewDefaultUISize : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gSavedSettings.setF32("UIScaleFactor", 1.0f);
		gSavedSettings.setBOOL("UIAutoScale", FALSE);	
		gViewerWindow->reshape(gViewerWindow->getWindowWidthRaw(), gViewerWindow->getWindowHeightRaw());
		return true;
	}
};

class LLViewToggleUI : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if(gAgentCamera.getCameraMode() != CAMERA_MODE_MOUSELOOK)
		{
			LLNotification::Params params("ConfirmHideUI");
			params.functor.function(boost::bind(&LLViewToggleUI::confirm, this, _1, _2));
			LLSD substitutions;
			// <FS:Ansariel> Notification not showing if hiding the UI
//#if LL_DARWIN
//			substitutions["SHORTCUT"] = "Cmd+Shift+U";
//#else
//			substitutions["SHORTCUT"] = "Ctrl+Shift+U";
//#endif
			substitutions["SHORTCUT"] = "Alt+Shift+U";
			// </FS:Ansariel>
			params.substitutions = substitutions;
			if (!gSavedSettings.getBOOL("HideUIControls"))
			{
				// hiding, so show notification
				LLNotifications::instance().add(params);
			}
			else
			{
				LLNotifications::instance().forceResponse(params, 0);
			}
		}
		return true;
	}

	void confirm(const LLSD& notification, const LLSD& response)
	{
		S32 option = LLNotificationsUtil::getSelectedOption(notification, response);

		if (option == 0) // OK
		{
			gViewerWindow->setUIVisibility(gSavedSettings.getBOOL("HideUIControls"));
			LLPanelStandStopFlying::getInstance()->setVisible(gSavedSettings.getBOOL("HideUIControls"));
			gSavedSettings.setBOOL("HideUIControls",!gSavedSettings.getBOOL("HideUIControls"));
		}
	}
};

// <FS:Ansariel> Notification not showing if hiding the UI
class LLViewCheckToggleUI : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return gViewerWindow->getUIVisibility();
	}
};
// </FS:Ansariel>

void handle_duplicate_in_place(void*)
{
	LL_INFOS() << "handle_duplicate_in_place" << LL_ENDL;

	LLVector3 offset(0.f, 0.f, 0.f);
	LLSelectMgr::getInstance()->selectDuplicate(offset, TRUE);
}

/* dead code 30-apr-2008
void handle_deed_object_to_group(void*)
{
	LLUUID group_id;
	
	LLSelectMgr::getInstance()->selectGetGroup(group_id);
	LLSelectMgr::getInstance()->sendOwner(LLUUID::null, group_id, FALSE);
	LLViewerStats::getInstance()->incStat(LLViewerStats::ST_RELEASE_COUNT);
}

BOOL enable_deed_object_to_group(void*)
{
	if(LLSelectMgr::getInstance()->getSelection()->isEmpty()) return FALSE;
	LLPermissions perm;
	LLUUID group_id;

	if (LLSelectMgr::getInstance()->selectGetGroup(group_id) &&
		gAgent.hasPowerInGroup(group_id, GP_OBJECT_DEED) &&
		LLSelectMgr::getInstance()->selectGetPermissions(perm) &&
		perm.deedToGroup(gAgent.getID(), group_id))
	{
		return TRUE;
	}
	return FALSE;
}

*/


/*
 * No longer able to support viewer side manipulations in this way
 *
void god_force_inv_owner_permissive(LLViewerObject* object,
									LLInventoryObject::object_list_t* inventory,
									S32 serial_num,
									void*)
{
	typedef std::vector<LLPointer<LLViewerInventoryItem> > item_array_t;
	item_array_t items;

	LLInventoryObject::object_list_t::const_iterator inv_it = inventory->begin();
	LLInventoryObject::object_list_t::const_iterator inv_end = inventory->end();
	for ( ; inv_it != inv_end; ++inv_it)
	{
		if(((*inv_it)->getType() != LLAssetType::AT_CATEGORY))
		{
			LLInventoryObject* obj = *inv_it;
			LLPointer<LLViewerInventoryItem> new_item = new LLViewerInventoryItem((LLViewerInventoryItem*)obj);
			LLPermissions perm(new_item->getPermissions());
			perm.setMaskBase(PERM_ALL);
			perm.setMaskOwner(PERM_ALL);
			new_item->setPermissions(perm);
			items.push_back(new_item);
		}
	}
	item_array_t::iterator end = items.end();
	item_array_t::iterator it;
	for(it = items.begin(); it != end; ++it)
	{
		// since we have the inventory item in the callback, it should not
		// invalidate iteration through the selection manager.
		object->updateInventory((*it), TASK_INVENTORY_ITEM_KEY, false);
	}
}
*/

void handle_object_owner_permissive(void*)
{
	// only send this if they're a god.
	if(gAgent.isGodlike())
	{
		// do the objects.
		LLSelectMgr::getInstance()->selectionSetObjectPermissions(PERM_BASE, TRUE, PERM_ALL, TRUE);
		LLSelectMgr::getInstance()->selectionSetObjectPermissions(PERM_OWNER, TRUE, PERM_ALL, TRUE);
	}
}

void handle_object_owner_self(void*)
{
	// only send this if they're a god.
	if(gAgent.isGodlike())
	{
		LLSelectMgr::getInstance()->sendOwner(gAgent.getID(), gAgent.getGroupID(), TRUE);
	}
}

// Shortcut to set owner permissions to not editable.
void handle_object_lock(void*)
{
	LLSelectMgr::getInstance()->selectionSetObjectPermissions(PERM_OWNER, FALSE, PERM_MODIFY);
}

void handle_object_asset_ids(void*)
{
	// only send this if they're a god.
	if (gAgent.isGodlike())
	{
		LLSelectMgr::getInstance()->sendGodlikeRequest("objectinfo", "assetids");
	}
}

void handle_force_parcel_owner_to_me(void*)
{
	LLViewerParcelMgr::getInstance()->sendParcelGodForceOwner( gAgent.getID() );
}

void handle_force_parcel_to_content(void*)
{
	LLViewerParcelMgr::getInstance()->sendParcelGodForceToContent();
}

void handle_claim_public_land(void*)
{
	if (LLViewerParcelMgr::getInstance()->getSelectionRegion() != gAgent.getRegion())
	{
		LLNotificationsUtil::add("ClaimPublicLand");
		return;
	}

	LLVector3d west_south_global;
	LLVector3d east_north_global;
	LLViewerParcelMgr::getInstance()->getSelection(west_south_global, east_north_global);
	LLVector3 west_south = gAgent.getPosAgentFromGlobal(west_south_global);
	LLVector3 east_north = gAgent.getPosAgentFromGlobal(east_north_global);

	LLMessageSystem* msg = gMessageSystem;
	msg->newMessage("GodlikeMessage");
	msg->nextBlock("AgentData");
	msg->addUUID("AgentID", gAgent.getID());
	msg->addUUID("SessionID", gAgent.getSessionID());
	msg->addUUIDFast(_PREHASH_TransactionID, LLUUID::null); //not used
	msg->nextBlock("MethodData");
	msg->addString("Method", "claimpublicland");
	msg->addUUID("Invoice", LLUUID::null);
	std::string buffer;
	buffer = llformat( "%f", west_south.mV[VX]);
	msg->nextBlock("ParamList");
	msg->addString("Parameter", buffer);
	buffer = llformat( "%f", west_south.mV[VY]);
	msg->nextBlock("ParamList");
	msg->addString("Parameter", buffer);
	buffer = llformat( "%f", east_north.mV[VX]);
	msg->nextBlock("ParamList");
	msg->addString("Parameter", buffer);
	buffer = llformat( "%f", east_north.mV[VY]);
	msg->nextBlock("ParamList");
	msg->addString("Parameter", buffer);
	gAgent.sendReliableMessage();
}



// HACK for easily testing new avatar geometry
void handle_god_request_avatar_geometry(void *)
{
	if (gAgent.isGodlike())
	{
		LLSelectMgr::getInstance()->sendGodlikeRequest("avatar toggle", "");
	}
}

static bool get_derezzable_objects(
	EDeRezDestination dest,
	std::string& error,
	LLViewerRegion*& first_region,
	std::vector<LLViewerObjectPtr>* derez_objectsp,
	bool only_check = false)
{
	bool found = false;

	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
	
	if (derez_objectsp)
		derez_objectsp->reserve(selection->getRootObjectCount());

	// Check conditions that we can't deal with, building a list of
	// everything that we'll actually be derezzing.
	for (LLObjectSelection::valid_root_iterator iter = selection->valid_root_begin();
		 iter != selection->valid_root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* object = node->getObject();
		LLViewerRegion* region = object->getRegion();
		if (!first_region)
		{
			first_region = region;
		}
		else
		{
			if(region != first_region)
			{
				// Derez doesn't work at all if the some of the objects
				// are in regions besides the first object selected.
				
				// ...crosses region boundaries
				error = "AcquireErrorObjectSpan";
				break;
			}
		}
		if (object->isAvatar())
		{
			// ...don't acquire avatars
			continue;
		}

		// If AssetContainers are being sent back, they will appear as 
		// boxes in the owner's inventory.
		if (object->getNVPair("AssetContainer")
			&& dest != DRD_RETURN_TO_OWNER)
		{
			// this object is an asset container, derez its contents, not it
			LL_WARNS() << "Attempt to derez deprecated AssetContainer object type not supported." << LL_ENDL;
			/*
			object->requestInventory(container_inventory_arrived, 
				(void *)(BOOL)(DRD_TAKE_INTO_AGENT_INVENTORY == dest));
			*/
			continue;
		}
		BOOL can_derez_current = FALSE;
		switch(dest)
		{
		case DRD_TAKE_INTO_AGENT_INVENTORY:
		case DRD_TRASH:
			if (!object->isPermanentEnforced() &&
				((node->mPermissions->allowTransferTo(gAgent.getID()) && object->permModify())
				|| (node->allowOperationOnNode(PERM_OWNER, GP_OBJECT_MANIPULATE))))
			{
				can_derez_current = TRUE;
			}
			break;

		case DRD_RETURN_TO_OWNER:
			if(!object->isAttachment())
			{
				can_derez_current = TRUE;
			}
			break;

		default:
			if((node->mPermissions->allowTransferTo(gAgent.getID())
				&& object->permCopy())
			   || gAgent.isGodlike())
			{
				can_derez_current = TRUE;
			}
			break;
		}
		if(can_derez_current)
		{
			found = true;

			if (only_check)
				// one found, no need to traverse to the end
				break;

			if (derez_objectsp)
				derez_objectsp->push_back(object);

		}
	}

	return found;
}

static bool can_derez(EDeRezDestination dest)
{
	LLViewerRegion* first_region = NULL;
	std::string error;
	return get_derezzable_objects(dest, error, first_region, NULL, true);
}

static void derez_objects(
	EDeRezDestination dest,
	const LLUUID& dest_id,
	LLViewerRegion*& first_region,
	std::string& error,
	std::vector<LLViewerObjectPtr>* objectsp)
{
	std::vector<LLViewerObjectPtr> derez_objects;

	if (!objectsp) // if objects to derez not specified
	{
		// get them from selection
		if (!get_derezzable_objects(dest, error, first_region, &derez_objects, false))
		{
			LL_WARNS() << "No objects to derez" << LL_ENDL;
			return;
		}

		objectsp = &derez_objects;
	}


	if(gAgentCamera.cameraMouselook())
	{
		gAgentCamera.changeCameraToDefault();
	}

	// This constant is based on (1200 - HEADER_SIZE) / 4 bytes per
	// root.  I lopped off a few (33) to provide a bit
	// pad. HEADER_SIZE is currently 67 bytes, most of which is UUIDs.
	// This gives us a maximum of 63500 root objects - which should
	// satisfy anybody.
	const S32 MAX_ROOTS_PER_PACKET = 250;
	const S32 MAX_PACKET_COUNT = 254;
	F32 packets = ceil((F32)objectsp->size() / (F32)MAX_ROOTS_PER_PACKET);
	if(packets > (F32)MAX_PACKET_COUNT)
	{
		error = "AcquireErrorTooManyObjects";
	}

	if(error.empty() && objectsp->size() > 0)
	{
		U8 d = (U8)dest;
		LLUUID tid;
		tid.generate();
		U8 packet_count = (U8)packets;
		S32 object_index = 0;
		S32 objects_in_packet = 0;
		LLMessageSystem* msg = gMessageSystem;
		for(U8 packet_number = 0;
			packet_number < packet_count;
			++packet_number)
		{
			msg->newMessageFast(_PREHASH_DeRezObject);
			msg->nextBlockFast(_PREHASH_AgentData);
			msg->addUUIDFast(_PREHASH_AgentID, gAgent.getID());
			msg->addUUIDFast(_PREHASH_SessionID, gAgent.getSessionID());
			msg->nextBlockFast(_PREHASH_AgentBlock);
			msg->addUUIDFast(_PREHASH_GroupID, gAgent.getGroupID());
			msg->addU8Fast(_PREHASH_Destination, d);	
			msg->addUUIDFast(_PREHASH_DestinationID, dest_id);
			msg->addUUIDFast(_PREHASH_TransactionID, tid);
			msg->addU8Fast(_PREHASH_PacketCount, packet_count);
			msg->addU8Fast(_PREHASH_PacketNumber, packet_number);
			objects_in_packet = 0;
			while((object_index < objectsp->size())
				  && (objects_in_packet++ < MAX_ROOTS_PER_PACKET))

			{
				LLViewerObject* object = objectsp->at(object_index++);
				msg->nextBlockFast(_PREHASH_ObjectData);
				msg->addU32Fast(_PREHASH_ObjectLocalID, object->getLocalID());
				// VEFFECT: DerezObject
				LLHUDEffectSpiral* effectp = (LLHUDEffectSpiral*)LLHUDManager::getInstance()->createViewerEffect(LLHUDObject::LL_HUD_EFFECT_POINT, TRUE);
				effectp->setPositionGlobal(object->getPositionGlobal());
				effectp->setColor(LLColor4U(gAgent.getEffectColor()));
			}
			msg->sendReliable(first_region->getHost());
		}
		make_ui_sound("UISndObjectRezOut");

		// Busy count decremented by inventory update, so only increment
		// if will be causing an update.
		if (dest != DRD_RETURN_TO_OWNER)
		{
			gViewerWindow->getWindow()->incBusyCount();
		}
	}
	else if(!error.empty())
	{
		LLNotificationsUtil::add(error);
	}
}

static void derez_objects(EDeRezDestination dest, const LLUUID& dest_id)
{
	LLViewerRegion* first_region = NULL;
	std::string error;
	derez_objects(dest, dest_id, first_region, error, NULL);
}

void handle_take_copy()
{
	if (LLSelectMgr::getInstance()->getSelection()->isEmpty()) return;

// [RLVa:KB] - Checked: 2010-03-07 (RLVa-1.2.0c) | Modified: RLVa-1.2.0a
	if ( (rlv_handler_t::isEnabled()) && (!RlvActions::canStand()) )
	{
		// Allow only if the avie isn't sitting on any of the selected objects
		LLObjectSelectionHandle hSel = LLSelectMgr::getInstance()->getSelection();
		RlvSelectIsSittingOn f(gAgentAvatarp);
		if ( (hSel.notNull()) && (hSel->getFirstRootNode(&f, TRUE) != NULL) )
			return;
	}
// [/RLVa:KB]

	const LLUUID category_id = gInventory.findCategoryUUIDForType(LLFolderType::FT_OBJECT);
	derez_objects(DRD_ACQUIRE_TO_AGENT_INVENTORY, category_id);
}

// You can return an object to its owner if it is on your land.
class LLObjectReturn : public view_listener_t
{
public:
	LLObjectReturn() : mFirstRegion(NULL) {}

private:
	bool handleEvent(const LLSD& userdata)
	{
		if (LLSelectMgr::getInstance()->getSelection()->isEmpty()) return true;
// [RLVa:KB] - Checked: 2010-03-24 (RLVa-1.4.0a) | Modified: RLVa-1.0.0b
		if ( (rlv_handler_t::isEnabled()) && (!rlvCanDeleteOrReturn()) ) return true;
// [/RLVa:KB]

		mObjectSelection = LLSelectMgr::getInstance()->getEditSelection();

		// Save selected objects, so that we still know what to return after the confirmation dialog resets selection.
		get_derezzable_objects(DRD_RETURN_TO_OWNER, mError, mFirstRegion, &mReturnableObjects);

		LLNotificationsUtil::add("ReturnToOwner", LLSD(), LLSD(), boost::bind(&LLObjectReturn::onReturnToOwner, this, _1, _2));
		return true;
	}

	bool onReturnToOwner(const LLSD& notification, const LLSD& response)
	{
		S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
		if (0 == option)
		{
			// Ignore category ID for this derez destination.
			derez_objects(DRD_RETURN_TO_OWNER, LLUUID::null, mFirstRegion, mError, &mReturnableObjects);
		}

		mReturnableObjects.clear();
		mError.clear();
		mFirstRegion = NULL;

		// drop reference to current selection
		mObjectSelection = NULL;
		return false;
	}

	LLObjectSelectionHandle mObjectSelection;

	std::vector<LLViewerObjectPtr> mReturnableObjects;
	std::string mError;
	LLViewerRegion* mFirstRegion;
};


// Allow return to owner if one or more of the selected items is
// over land you own.
class LLObjectEnableReturn : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (LLSelectMgr::getInstance()->getSelection()->isEmpty())
		{
			// Do not enable if nothing selected
			return false;
		}
// [RLVa:KB] - Checked: 2011-05-28 (RLVa-1.4.0a) | Modified: RLVa-1.4.0a
		if ( (rlv_handler_t::isEnabled()) && (!rlvCanDeleteOrReturn()) )
		{
			return false;
		}
// [/RLVa:KB]
#ifdef HACKED_GODLIKE_VIEWER
		bool new_value = true;
#else
		bool new_value = false;
		if (gAgent.isGodlike())
		{
			new_value = true;
		}
		else
		{
			new_value = can_derez(DRD_RETURN_TO_OWNER);
		}
#endif
		return new_value;
	}
};

void force_take_copy(void*)
{
	if (LLSelectMgr::getInstance()->getSelection()->isEmpty()) return;
	const LLUUID category_id = gInventory.findCategoryUUIDForType(LLFolderType::FT_OBJECT);
	derez_objects(DRD_FORCE_TO_GOD_INVENTORY, category_id);
}

void handle_take()
{
	// we want to use the folder this was derezzed from if it's
	// available. Otherwise, derez to the normal place.
//	if(LLSelectMgr::getInstance()->getSelection()->isEmpty())
// [RLVa:KB] - Checked: 2010-03-24 (RLVa-1.2.0e) | Modified: RLVa-1.0.0b
	if ( (LLSelectMgr::getInstance()->getSelection()->isEmpty()) || ((rlv_handler_t::isEnabled()) && (!rlvCanDeleteOrReturn())) )
// [/RLVa:KB]
	{
		return;
	}

	BOOL you_own_everything = TRUE;
	BOOL locked_but_takeable_object = FALSE;
	LLUUID category_id;
	
	for (LLObjectSelection::root_iterator iter = LLSelectMgr::getInstance()->getSelection()->root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* object = node->getObject();
		if(object)
		{
			if(!object->permYouOwner())
			{
				you_own_everything = FALSE;
			}

			if(!object->permMove())
			{
				locked_but_takeable_object = TRUE;
			}
		}
		if(node->mFolderID.notNull())
		{
			if(category_id.isNull())
			{
				category_id = node->mFolderID;
				LL_DEBUGS("HandleTake") << "Node destination folder ID = " << category_id.asString() << LL_ENDL;
			}
			else if(category_id != node->mFolderID)
			{
				// we have found two potential destinations. break out
				// now and send to the default location.
				category_id.setNull();
				LL_DEBUGS("HandleTake") << "Conflicting node destination folders - setting to null UUID" << LL_ENDL;
				break;
			}
		}
	}
	if(category_id.notNull())
	{
		LL_DEBUGS("HandleTake") << "Selected destination folder ID: " << category_id.asString() << " - checking if category exists in inventory model" << LL_ENDL;

		// there is an unambiguous destination. See if this agent has
		// such a location and it is not in the trash or library
		if(!gInventory.getCategory(category_id))
		{
			// nope, set to NULL.
			category_id.setNull();
			LL_DEBUGS("HandleTake") << "Destination folder not found in inventory model - setting to null UUID" << LL_ENDL;
		}
		if(category_id.notNull())
		{
		        // check trash
			const LLUUID trash = gInventory.findCategoryUUIDForType(LLFolderType::FT_TRASH);
			if(category_id == trash || gInventory.isObjectDescendentOf(category_id, trash))
			{
				category_id.setNull();
				LL_DEBUGS("HandleTake") << "Destination folder is descendent of trash folder - setting to null UUID" << LL_ENDL;
			}

			// check library
			if(gInventory.isObjectDescendentOf(category_id, gInventory.getLibraryRootFolderID()))
			{
				category_id.setNull();
				LL_DEBUGS("HandleTake") << "Destination folder is descendent of library folder - setting to null UUID" << LL_ENDL;
			}

		}
	}
	if(category_id.isNull())
	{
		category_id = gInventory.findCategoryUUIDForType(LLFolderType::FT_OBJECT);
		LL_DEBUGS("HandleTake") << "Destination folder = null UUID - determined default category: " << category_id.asString() << LL_ENDL;
	}
	LLSD payload;
	payload["folder_id"] = category_id;
	LL_DEBUGS("HandleTake") << "Final destination folder UUID being sent to sim: " << category_id.asString() << LL_ENDL;

	LLNotification::Params params("ConfirmObjectTakeLock");
	params.payload(payload);
	// MAINT-290
	// Reason: Showing the confirmation dialog resets object selection,	thus there is nothing to derez.
	// Fix: pass selection to the confirm_take, so that selection doesn't "die" after confirmation dialog is opened
	params.functor.function(boost::bind(confirm_take, _1, _2, LLSelectMgr::instance().getSelection()));

	if(locked_but_takeable_object ||
	   !you_own_everything)
	{
		if(locked_but_takeable_object && you_own_everything)
		{
			params.name("ConfirmObjectTakeLock");
		}
		else if(!locked_but_takeable_object && !you_own_everything)
		{
			params.name("ConfirmObjectTakeNoOwn");
		}
		else
		{
			params.name("ConfirmObjectTakeLockNoOwn");
		}
	
		LLNotifications::instance().add(params);
	}
	else
	{
		LLNotifications::instance().forceResponse(params, 0);
	}
}

void handle_object_show_inspector()
{
	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
	LLViewerObject* objectp = selection->getFirstRootObject(TRUE);
 	if (!objectp)
 	{
 		return;
 	}

	LLSD params;
	params["object_id"] = objectp->getID();
	LLFloaterReg::showInstance("inspect_object", params);
}

void handle_avatar_show_inspector()
{
	LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
	if(avatar)
	{
		LLSD params;
		params["avatar_id"] = avatar->getID();
		LLFloaterReg::showInstance("inspect_avatar", params);
	}
}



bool confirm_take(const LLSD& notification, const LLSD& response, LLObjectSelectionHandle selection_handle)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if(enable_take() && (option == 0))
	{
		derez_objects(DRD_TAKE_INTO_AGENT_INVENTORY, notification["payload"]["folder_id"].asUUID());
	}
	return false;
}

// You can take an item when it is public and transferrable, or when
// you own it. We err on the side of enabling the item when at least
// one item selected can be copied to inventory.
BOOL enable_take()
{
//	if (sitting_on_selection())
// [RLVa:KB] - Checked: 2010-03-24 (RLVa-1.2.0e) | Modified: RLVa-1.0.0b
	if ( (sitting_on_selection()) || ((rlv_handler_t::isEnabled()) && (!rlvCanDeleteOrReturn())) )
// [/RLVa:KB]
	{
		return FALSE;
	}

	for (LLObjectSelection::valid_root_iterator iter = LLSelectMgr::getInstance()->getSelection()->valid_root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->valid_root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* object = node->getObject();
		if (object->isAvatar())
		{
			// ...don't acquire avatars
			continue;
		}

#ifdef HACKED_GODLIKE_VIEWER
		return TRUE;
#else
# ifdef TOGGLE_HACKED_GODLIKE_VIEWER
		if (LLGridManager::getInstance()->isInSLBeta() 
            && gAgent.isGodlike())
		{
			return TRUE;
		}
# endif
		if(!object->isPermanentEnforced() &&
			((node->mPermissions->allowTransferTo(gAgent.getID())
			&& object->permModify())
			|| (node->mPermissions->getOwner() == gAgent.getID())))
		{
			return !object->isAttachment();
		}
#endif
	}
	return FALSE;
}


void handle_buy_or_take()
{
	if (LLSelectMgr::getInstance()->getSelection()->isEmpty())
	{
		return;
	}

	if (is_selection_buy_not_take())
	{
		S32 total_price = selection_price();

		if (total_price <= gStatusBar->getBalance() || total_price == 0)
		{
			handle_buy();
		}
		else
		{
			LLStringUtil::format_map_t args;
			args["AMOUNT"] = llformat("%d", total_price);
			LLBuyCurrencyHTML::openCurrencyFloater( LLTrans::getString( "BuyingCosts", args ), total_price );
		}
	}
	else
	{
		handle_take();
	}
}

bool visible_buy_object()
{
	return is_selection_buy_not_take() && enable_buy_object();
}

bool visible_take_object()
{
	return !is_selection_buy_not_take() && enable_take();
}

bool tools_visible_buy_object()
{
	return is_selection_buy_not_take();
}

bool tools_visible_take_object()
{
	return !is_selection_buy_not_take();
}

bool enable_how_to_visible(const LLSD& param)
{
	LLFloaterWebContent::Params p;
	p.target = "__help_how_to";
	return LLFloaterReg::instanceVisible("how_to", p);
}

class LLToolsEnableBuyOrTake : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool is_buy = is_selection_buy_not_take();
		bool new_value = is_buy ? enable_buy_object() : enable_take();
		return new_value;
	}
};

// This is a small helper function to determine if we have a buy or a
// take in the selection. This method is to help with the aliasing
// problems of putting buy and take in the same pie menu space. After
// a fair amont of discussion, it was determined to prefer buy over
// take. The reasoning follows from the fact that when users walk up
// to buy something, they will click on one or more items. Thus, if
// anything is for sale, it becomes a buy operation, and the server
// will group all of the buy items, and copyable/modifiable items into
// one package and give the end user as much as the permissions will
// allow. If the user wanted to take something, they will select fewer
// and fewer items until only 'takeable' items are left. The one
// exception is if you own everything in the selection that is for
// sale, in this case, you can't buy stuff from yourself, so you can
// take it.
// return value = TRUE if selection is a 'buy'.
//                FALSE if selection is a 'take'
BOOL is_selection_buy_not_take()
{
	for (LLObjectSelection::root_iterator iter = LLSelectMgr::getInstance()->getSelection()->root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* obj = node->getObject();
		if(obj && !(obj->permYouOwner()) && (node->mSaleInfo.isForSale()))
		{
			// you do not own the object and it is for sale, thus,
			// it's a buy
			return TRUE;
		}
	}
	return FALSE;
}

S32 selection_price()
{
	S32 total_price = 0;
	for (LLObjectSelection::root_iterator iter = LLSelectMgr::getInstance()->getSelection()->root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* obj = node->getObject();
		if(obj && !(obj->permYouOwner()) && (node->mSaleInfo.isForSale()))
		{
			// you do not own the object and it is for sale.
			// Add its price.
			total_price += node->mSaleInfo.getSalePrice();
		}
	}

	return total_price;
}
/*
bool callback_show_buy_currency(const LLSD& notification, const LLSD& response)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (0 == option)
	{
		LL_INFOS() << "Loading page " << LLNotifications::instance().getGlobalString("BUY_CURRENCY_URL") << LL_ENDL;
		LLWeb::loadURL(LLNotifications::instance().getGlobalString("BUY_CURRENCY_URL"));
	}
	return false;
}
*/

void show_buy_currency(const char* extra)
{
	// Don't show currency web page for branded clients.
/*
	std::ostringstream mesg;
	if (extra != NULL)
	{	
		mesg << extra << "\n \n";
	}
	mesg << "Go to " << LLNotifications::instance().getGlobalString("BUY_CURRENCY_URL")<< "\nfor information on purchasing currency?";
*/
	LLSD args;
	if (extra != NULL)
	{
		args["EXTRA"] = extra;
	}
	LLNotificationsUtil::add("PromptGoToCurrencyPage", args);//, LLSD(), callback_show_buy_currency);
}

void handle_buy()
{
	if (LLSelectMgr::getInstance()->getSelection()->isEmpty()) return;

	LLSaleInfo sale_info;
	BOOL valid = LLSelectMgr::getInstance()->selectGetSaleInfo(sale_info);
	if (!valid) return;

	S32 price = sale_info.getSalePrice();
	
	if (price > 0 && price > gStatusBar->getBalance())
	{
		LLStringUtil::format_map_t args;
		args["AMOUNT"] = llformat("%d", price);
		LLBuyCurrencyHTML::openCurrencyFloater( LLTrans::getString("this_object_costs", args), price );
		return;
	}

	if (sale_info.getSaleType() == LLSaleInfo::FS_CONTENTS)
	{
		handle_buy_contents(sale_info);
	}
	else
	{
		handle_buy_object(sale_info);
	}
}

bool anyone_copy_selection(LLSelectNode* nodep)
{
	bool perm_copy = (bool)(nodep->getObject()->permCopy());
	bool all_copy = (bool)(nodep->mPermissions->getMaskEveryone() & PERM_COPY);
	return perm_copy && all_copy;
}

bool for_sale_selection(LLSelectNode* nodep)
{
	return nodep->mSaleInfo.isForSale()
		&& nodep->mPermissions->getMaskOwner() & PERM_TRANSFER
		&& (nodep->mPermissions->getMaskOwner() & PERM_COPY
			|| nodep->mSaleInfo.getSaleType() != LLSaleInfo::FS_COPY);
}

BOOL sitting_on_selection()
{
	LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
	if (!node)
	{
		return FALSE;
	}

	if (!node->mValid)
	{
		return FALSE;
	}

	LLViewerObject* root_object = node->getObject();
	if (!root_object)
	{
		return FALSE;
	}

	// Need to determine if avatar is sitting on this object
	if (!isAgentAvatarValid()) return FALSE;

	return (gAgentAvatarp->isSitting() && gAgentAvatarp->getRoot() == root_object);
}

class LLToolsSaveToObjectInventory : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
		if(node && (node->mValid) && (!node->mFromTaskID.isNull()))
		{
			// *TODO: check to see if the fromtaskid object exists.
			derez_objects(DRD_SAVE_INTO_TASK_INVENTORY, node->mFromTaskID);
		}
		return true;
	}
};

class LLToolsEnablePathfinding : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return (LLPathfindingManager::getInstance() != NULL) && LLPathfindingManager::getInstance()->isPathfindingEnabledForCurrentRegion();
	}
};

class LLToolsEnablePathfindingView : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return (LLPathfindingManager::getInstance() != NULL) && LLPathfindingManager::getInstance()->isPathfindingEnabledForCurrentRegion() && LLPathfindingManager::getInstance()->isPathfindingViewEnabled();
	}
};

class LLToolsDoPathfindingRebakeRegion : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool hasPathfinding = (LLPathfindingManager::getInstance() != NULL);

		if (hasPathfinding)
		{
			LLMenuOptionPathfindingRebakeNavmesh::getInstance()->sendRequestRebakeNavmesh();
		}

		return hasPathfinding;
	}
};

class LLToolsEnablePathfindingRebakeRegion : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool returnValue = false;

		if (LLPathfindingManager::getInstance() != NULL)
		{
			LLMenuOptionPathfindingRebakeNavmesh *rebakeInstance = LLMenuOptionPathfindingRebakeNavmesh::getInstance();
			returnValue = (rebakeInstance->canRebakeRegion() &&
				(rebakeInstance->getMode() == LLMenuOptionPathfindingRebakeNavmesh::kRebakeNavMesh_Available));
		}
		return returnValue;
	}
};

// Round the position of all root objects to the grid
class LLToolsSnapObjectXY : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		F64 snap_size = (F64)gSavedSettings.getF32("GridResolution");

		for (LLObjectSelection::root_iterator iter = LLSelectMgr::getInstance()->getSelection()->root_begin();
			 iter != LLSelectMgr::getInstance()->getSelection()->root_end(); iter++)
		{
			LLSelectNode* node = *iter;
			LLViewerObject* obj = node->getObject();
			if (obj->permModify())
			{
				LLVector3d pos_global = obj->getPositionGlobal();
				F64 round_x = fmod(pos_global.mdV[VX], snap_size);
				if (round_x < snap_size * 0.5)
				{
					// closer to round down
					pos_global.mdV[VX] -= round_x;
				}
				else
				{
					// closer to round up
					pos_global.mdV[VX] -= round_x;
					pos_global.mdV[VX] += snap_size;
				}

				F64 round_y = fmod(pos_global.mdV[VY], snap_size);
				if (round_y < snap_size * 0.5)
				{
					pos_global.mdV[VY] -= round_y;
				}
				else
				{
					pos_global.mdV[VY] -= round_y;
					pos_global.mdV[VY] += snap_size;
				}

				obj->setPositionGlobal(pos_global, FALSE);
			}
		}
		LLSelectMgr::getInstance()->sendMultipleUpdate(UPD_POSITION);
		return true;
	}
};

// Determine if the option to cycle between linked prims is shown
class LLToolsEnableSelectNextPart : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
        bool new_value = (!LLSelectMgr::getInstance()->getSelection()->isEmpty()
                          && (gSavedSettings.getBOOL("EditLinkedParts")
                              || LLToolFace::getInstance() == LLToolMgr::getInstance()->getCurrentTool()));
		return new_value;
	}
};

// Cycle selection through linked children or/and faces in selected object.
// FIXME: Order of children list is not always the same as sim's idea of link order. This may confuse
// resis. Need link position added to sim messages to address this.
class LLToolsSelectNextPartFace : public view_listener_t
{
    bool handleEvent(const LLSD& userdata)
    {
        bool cycle_faces = LLToolFace::getInstance() == LLToolMgr::getInstance()->getCurrentTool();
        bool cycle_linked = gSavedSettings.getBOOL("EditLinkedParts");

        if (!cycle_faces && !cycle_linked)
        {
            // Nothing to do
            return true;
        }

        bool fwd = (userdata.asString() == "next");
        bool prev = (userdata.asString() == "previous");
        bool ifwd = (userdata.asString() == "includenext");
        bool iprev = (userdata.asString() == "includeprevious");

        LLViewerObject* to_select = NULL;
        bool restart_face_on_part = !cycle_faces;
        S32 new_te = 0;

        if (cycle_faces)
        {
            // Cycle through faces of current selection, if end is reached, swithc to next part (if present)
            LLSelectNode* nodep = LLSelectMgr::getInstance()->getSelection()->getFirstNode();
            if (!nodep) return false;
            to_select = nodep->getObject();
            if (!to_select) return false;

            S32 te_count = to_select->getNumTEs();
            S32 selected_te = nodep->getLastOperatedTE();

            if (fwd || ifwd)
            {
                if (selected_te < 0)
                {
                    new_te = 0;
                }
                else if (selected_te + 1 < te_count)
                {
                    // select next face
                    new_te = selected_te + 1;
                }
                else
                {
                    // restart from first face on next part
                    restart_face_on_part = true;
                }
            }
            else if (prev || iprev)
            {
                if (selected_te > te_count)
                {
                    new_te = te_count - 1;
                }
                else if (selected_te - 1 >= 0)
                {
                    // select previous face
                    new_te = selected_te - 1;
                }
                else
                {
                    // restart from last face on next part
                    restart_face_on_part = true;
                }
            }
        }

		S32 object_count = LLSelectMgr::getInstance()->getSelection()->getObjectCount();
		if (cycle_linked && object_count && restart_face_on_part)
		{
			LLViewerObject* selected = LLSelectMgr::getInstance()->getSelection()->getFirstObject();
			if (selected && selected->getRootEdit())
			{
				LLViewerObject::child_list_t children = selected->getRootEdit()->getChildren();
				children.push_front(selected->getRootEdit());	// need root in the list too

				for (LLViewerObject::child_list_t::iterator iter = children.begin(); iter != children.end(); ++iter)
				{
					if ((*iter)->isSelected())
					{
						if (object_count > 1 && (fwd || prev))	// multiple selection, find first or last selected if not include
						{
							to_select = *iter;
							if (fwd)
							{
								// stop searching if going forward; repeat to get last hit if backward
								break;
							}
						}
						else if ((object_count == 1) || (ifwd || iprev))	// single selection or include
						{
							if (fwd || ifwd)
							{
								++iter;
								while (iter != children.end() && ((*iter)->isAvatar() || (ifwd && (*iter)->isSelected())))
								{
									++iter;	// skip sitting avatars and selected if include
								}
							}
							else // backward
							{
								iter = (iter == children.begin() ? children.end() : iter);
								--iter;
								while (iter != children.begin() && ((*iter)->isAvatar() || (iprev && (*iter)->isSelected())))
								{
									--iter;	// skip sitting avatars and selected if include
								}
							}
							iter = (iter == children.end() ? children.begin() : iter);
							to_select = *iter;
							break;
						}
					}
				}
			}
		}

        if (to_select)
        {
            if (gFocusMgr.childHasKeyboardFocus(gFloaterTools))
            {
                gFocusMgr.setKeyboardFocus(NULL);	// force edit toolbox to commit any changes
            }
            if (fwd || prev)
            {
                LLSelectMgr::getInstance()->deselectAll();
            }
            if (cycle_faces)
            {
                if (restart_face_on_part)
                {
                    if (fwd || ifwd)
                    {
                        new_te = 0;
                    }
                    else
                    {
                        new_te = to_select->getNumTEs() - 1;
                    }
                }
                LLSelectMgr::getInstance()->addAsIndividual(to_select, new_te, FALSE);
            }
            else
            {
                LLSelectMgr::getInstance()->selectObjectOnly(to_select);
            }
            return true;
        }
		return true;
	}
};

class LLToolsStopAllAnimations : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// <FS:Ansariel> Allow legacy stop animations without revoking script permissions
		//gAgent.stopCurrentAnimations();
		std::string param = userdata.asString();
		if (param.empty() || param == "stoprevoke")
		{
			gAgent.stopCurrentAnimations();
		}
		else if (param == "stop")
		{
			gAgent.stopCurrentAnimations(true);
		}
		// </FS:Ansariel>
		return true;
	}
};

class LLToolsReleaseKeys : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-04-19 (RLVa-1.2.0f) | Modified: RLVa-1.0.5a
		if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) )
			return true;
// [/RLVa:KB]

		gAgent.forceReleaseControls();
		return true;
	}
};

class LLToolsEnableReleaseKeys : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-04-19 (RLVa-1.2.0f) | Modified: RLVa-1.0.5a
		return (gAgent.anyControlGrabbed()) && 
			( (!rlv_handler_t::isEnabled()) || (!gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) );
// [/RLVa:KB]
//		return gAgent.anyControlGrabbed();
	}
};


class LLEditEnableCut : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canCut();
		return new_value;
	}
};

class LLEditCut : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler )
		{
			LLEditMenuHandler::gEditMenuHandler->cut();
		}
		return true;
	}
};

class LLEditEnableCopy : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canCopy();
		return new_value;
	}
};

class LLEditCopy : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler )
		{
			LLEditMenuHandler::gEditMenuHandler->copy();
		}
		return true;
	}
};

class LLEditEnablePaste : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canPaste();
		return new_value;
	}
};

class LLEditPaste : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler )
		{
			LLEditMenuHandler::gEditMenuHandler->paste();
		}
		return true;
	}
};

class LLEditEnableDelete : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canDoDelete();
		return new_value;
	}
};

class LLEditDelete : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// If a text field can do a deletion, it gets precedence over deleting
		// an object in the world.
		if( LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canDoDelete())
		{
			LLEditMenuHandler::gEditMenuHandler->doDelete();
		}

		// and close any pie/context menus when done
		gMenuHolder->hideMenus();

		// When deleting an object we may not actually be done
		// Keep selection so we know what to delete when confirmation is needed about the delete
		gMenuObject->hide();
		return true;
	}
};

void handle_spellcheck_replace_with_suggestion(const LLUICtrl* ctrl, const LLSD& param)
{
	const LLContextMenu* menu = dynamic_cast<const LLContextMenu*>(ctrl->getParent());
	LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	if ( (!spellcheck_handler) || (!spellcheck_handler->getSpellCheck()) )
	{
		return;
	}

	U32 index = 0;
	if ( (!LLStringUtil::convertToU32(param.asString(), index)) || (index >= spellcheck_handler->getSuggestionCount()) )
	{
		return;
	}

	spellcheck_handler->replaceWithSuggestion(index);
}

bool visible_spellcheck_suggestion(LLUICtrl* ctrl, const LLSD& param)
{
	LLMenuItemGL* item = dynamic_cast<LLMenuItemGL*>(ctrl);
	const LLContextMenu* menu = (item) ? dynamic_cast<const LLContextMenu*>(item->getParent()) : NULL;
	const LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<const LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	if ( (!spellcheck_handler) || (!spellcheck_handler->getSpellCheck()) )
	{
		return false;
	}

	U32 index = 0;
	if ( (!LLStringUtil::convertToU32(param.asString(), index)) || (index >= spellcheck_handler->getSuggestionCount()) )
	{
		return false;
	}

	item->setLabel(spellcheck_handler->getSuggestion(index));
	return true;
}

void handle_spellcheck_add_to_dictionary(const LLUICtrl* ctrl)
{
	const LLContextMenu* menu = dynamic_cast<const LLContextMenu*>(ctrl->getParent());
	LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	if ( (spellcheck_handler) && (spellcheck_handler->canAddToDictionary()) )
	{
		spellcheck_handler->addToDictionary();
	}
}

bool enable_spellcheck_add_to_dictionary(const LLUICtrl* ctrl)
{
	const LLContextMenu* menu = dynamic_cast<const LLContextMenu*>(ctrl->getParent());
	const LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<const LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	return (spellcheck_handler) && (spellcheck_handler->canAddToDictionary());
}

void handle_spellcheck_add_to_ignore(const LLUICtrl* ctrl)
{
	const LLContextMenu* menu = dynamic_cast<const LLContextMenu*>(ctrl->getParent());
	LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	if ( (spellcheck_handler) && (spellcheck_handler->canAddToIgnore()) )
	{
		spellcheck_handler->addToIgnore();
	}
}

bool enable_spellcheck_add_to_ignore(const LLUICtrl* ctrl)
{
	const LLContextMenu* menu = dynamic_cast<const LLContextMenu*>(ctrl->getParent());
	const LLSpellCheckMenuHandler* spellcheck_handler = (menu) ? dynamic_cast<const LLSpellCheckMenuHandler*>(menu->getSpawningView()) : NULL;
	return (spellcheck_handler) && (spellcheck_handler->canAddToIgnore());
}

bool enable_object_return()
{
	return (!LLSelectMgr::getInstance()->getSelection()->isEmpty() &&
		(gAgent.isGodlike() || can_derez(DRD_RETURN_TO_OWNER)));
}

bool enable_object_delete()
{
	bool new_value = 
#ifdef HACKED_GODLIKE_VIEWER
	TRUE;
#else
# ifdef TOGGLE_HACKED_GODLIKE_VIEWER
	(LLGridManager::getInstance()->isInSLBeta()
     && gAgent.isGodlike()) ||
# endif
	LLSelectMgr::getInstance()->canDoDelete();
#endif
	return new_value;
}

class LLObjectsReturnPackage
{
public:
	LLObjectsReturnPackage() : mObjectSelection(), mReturnableObjects(), mError(),	mFirstRegion(NULL) {};
	~LLObjectsReturnPackage()
	{
		mObjectSelection.clear();
		mReturnableObjects.clear();
		mError.clear();
		mFirstRegion = NULL;
	};

	LLObjectSelectionHandle mObjectSelection;
	std::vector<LLViewerObjectPtr> mReturnableObjects;
	std::string mError;
	LLViewerRegion *mFirstRegion;
};

static void return_objects(LLObjectsReturnPackage *objectsReturnPackage, const LLSD& notification, const LLSD& response)
{
	if (LLNotificationsUtil::getSelectedOption(notification, response) == 0)
	{
		// Ignore category ID for this derez destination.
		derez_objects(DRD_RETURN_TO_OWNER, LLUUID::null, objectsReturnPackage->mFirstRegion, objectsReturnPackage->mError, &objectsReturnPackage->mReturnableObjects);
	}

	delete objectsReturnPackage;
}

void handle_object_return()
{
	if (!LLSelectMgr::getInstance()->getSelection()->isEmpty())
	{
		LLObjectsReturnPackage *objectsReturnPackage = new LLObjectsReturnPackage();
		objectsReturnPackage->mObjectSelection = LLSelectMgr::getInstance()->getEditSelection();

		// Save selected objects, so that we still know what to return after the confirmation dialog resets selection.
		get_derezzable_objects(DRD_RETURN_TO_OWNER, objectsReturnPackage->mError, objectsReturnPackage->mFirstRegion, &objectsReturnPackage->mReturnableObjects);

		LLNotificationsUtil::add("ReturnToOwner", LLSD(), LLSD(), boost::bind(&return_objects, objectsReturnPackage, _1, _2));
	}
}

void handle_object_delete()
{

		if (LLSelectMgr::getInstance())
		{
			LLSelectMgr::getInstance()->doDelete();
		}

		// and close any pie/context menus when done
		gMenuHolder->hideMenus();

		// When deleting an object we may not actually be done
		// Keep selection so we know what to delete when confirmation is needed about the delete
		gMenuObject->hide();
		return;
}

void handle_force_delete(void*)
{
	LLSelectMgr::getInstance()->selectForceDelete();
}

class LLViewEnableJoystickFlycam : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = (gSavedSettings.getBOOL("JoystickEnabled") && gSavedSettings.getBOOL("JoystickFlycamEnabled"));
		return new_value;
	}
};

class LLViewEnableLastChatter : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// *TODO: add check that last chatter is in range
		bool new_value = (gAgentCamera.cameraThirdPerson() && gAgent.getLastChatter().notNull());
		return new_value;
	}
};

class LLEditEnableDeselect : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canDeselect();
		return new_value;
	}
};

class LLEditDeselect : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler )
		{
			LLEditMenuHandler::gEditMenuHandler->deselect();
		}
		return true;
	}
};

class LLEditEnableSelectAll : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canSelectAll();
		return new_value;
	}
};


class LLEditSelectAll : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler )
		{
			LLEditMenuHandler::gEditMenuHandler->selectAll();
		}
		return true;
	}
};


class LLEditEnableUndo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canUndo();
		return new_value;
	}
};

class LLEditUndo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canUndo() )
		{
			LLEditMenuHandler::gEditMenuHandler->undo();
		}
		return true;
	}
};

class LLEditEnableRedo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canRedo();
		return new_value;
	}
};

class LLEditRedo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if( LLEditMenuHandler::gEditMenuHandler && LLEditMenuHandler::gEditMenuHandler->canRedo() )
		{
			LLEditMenuHandler::gEditMenuHandler->redo();
		}
		return true;
	}
};



void print_object_info(void*)
{
	LLSelectMgr::getInstance()->selectionDump();
}

void print_agent_nvpairs(void*)
{
	LLViewerObject *objectp;

	LL_INFOS() << "Agent Name Value Pairs" << LL_ENDL;

	objectp = gObjectList.findObject(gAgentID);
	if (objectp)
	{
		objectp->printNameValuePairs();
	}
	else
	{
		LL_INFOS() << "Can't find agent object" << LL_ENDL;
	}

	LL_INFOS() << "Camera at " << gAgentCamera.getCameraPositionGlobal() << LL_ENDL;
}

void show_debug_menus()
{
	// this might get called at login screen where there is no menu so only toggle it if one exists
	if ( gMenuBarView )
	{
		BOOL debug = gSavedSettings.getBOOL("UseDebugMenus");
		BOOL qamode = gSavedSettings.getBOOL("QAMode");

		gMenuBarView->setItemVisible("Advanced", debug);
// 		gMenuBarView->setItemEnabled("Advanced", debug); // Don't disable Advanced keyboard shortcuts when hidden

// [RLVa:KB] - Checked: 2011-08-16 (RLVa-1.4.0b) | Modified: RLVa-1.4.0b
		// NOTE: this is supposed to execute whether RLVa is enabled or not
		rlvMenuToggleVisible();
// [/RLVa:KB]
		
		gMenuBarView->setItemVisible("Debug", qamode);
		gMenuBarView->setItemEnabled("Debug", qamode);

		gMenuBarView->setItemVisible("Develop", qamode);
		gMenuBarView->setItemEnabled("Develop", qamode);

		// Server ('Admin') menu hidden when not in godmode.
		const bool show_server_menu = (gAgent.getGodLevel() > GOD_NOT || (debug && gAgent.getAdminOverride()));
		gMenuBarView->setItemVisible("Admin", show_server_menu);
		gMenuBarView->setItemEnabled("Admin", show_server_menu);
	}
	if (gLoginMenuBarView)
	{
		BOOL debug = gSavedSettings.getBOOL("UseDebugMenus");
		gLoginMenuBarView->setItemVisible("Debug", debug);
		gLoginMenuBarView->setItemEnabled("Debug", debug);
	}
}

void toggle_debug_menus(void*)
{
	BOOL visible = ! gSavedSettings.getBOOL("UseDebugMenus");
	gSavedSettings.setBOOL("UseDebugMenus", visible);
	show_debug_menus();
}

// LLUUID gExporterRequestID;
// std::string gExportDirectory;

// LLUploadDialog *gExportDialog = NULL;

// void handle_export_selected( void * )
// {
// 	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
// 	if (selection->isEmpty())
// 	{
// 		return;
// 	}
// 	LL_INFOS() << "Exporting selected objects:" << LL_ENDL;

// 	gExporterRequestID.generate();
// 	gExportDirectory = "";

// 	LLMessageSystem* msg = gMessageSystem;
// 	msg->newMessageFast(_PREHASH_ObjectExportSelected);
// 	msg->nextBlockFast(_PREHASH_AgentData);
// 	msg->addUUIDFast(_PREHASH_AgentID, gAgent.getID());
// 	msg->addUUIDFast(_PREHASH_RequestID, gExporterRequestID);
// 	msg->addS16Fast(_PREHASH_VolumeDetail, 4);

// 	for (LLObjectSelection::root_iterator iter = selection->root_begin();
// 		 iter != selection->root_end(); iter++)
// 	{
// 		LLSelectNode* node = *iter;
// 		LLViewerObject* object = node->getObject();
// 		msg->nextBlockFast(_PREHASH_ObjectData);
// 		msg->addUUIDFast(_PREHASH_ObjectID, object->getID());
// 		LL_INFOS() << "Object: " << object->getID() << LL_ENDL;
// 	}
// 	msg->sendReliable(gAgent.getRegion()->getHost());

// 	gExportDialog = LLUploadDialog::modalUploadDialog("Exporting selected objects...");
// }
//

// <FS:Ansariel> [FS Communication UI]
//class LLCommunicateNearbyChat : public view_listener_t
//{
//	bool handleEvent(const LLSD& userdata)
//	{
//		LLFloaterIMContainer* im_box = LLFloaterIMContainer::getInstance();
//		bool nearby_visible	= LLFloaterReg::getTypedInstance<LLFloaterIMNearbyChat>("nearby_chat")->isInVisibleChain();
//		if(nearby_visible && im_box->getSelectedSession() == LLUUID() && im_box->getConversationListItemSize() > 1)
//		{
//			im_box->selectNextorPreviousConversation(false);
//		}
//		else
//		{
//			LLFloaterReg::toggleInstanceOrBringToFront("nearby_chat");
//		}
//		return true;
//	}
//};
// </FS:Ansariel> [FS Communication UI]

class LLWorldSetHomeLocation : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// we just send the message and let the server check for failure cases
		// server will echo back a "Home position set." alert if it succeeds
		// and the home location screencapture happens when that alert is recieved
		gAgent.setStartPosition(START_LOCATION_ID_HOME);
		return true;
	}
};

class LLWorldTeleportHome : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gAgent.teleportHome();
		return true;
	}
};

class LLWorldAlwaysRun : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// as well as altering the default walk-vs-run state,
		// we also change the *current* walk-vs-run state.
		if (gAgent.getAlwaysRun())
		{
			gAgent.clearAlwaysRun();
//			gAgent.clearRunning();
			report_to_nearby_chat(LLTrans::getString("AlwaysRunDisabled"));
		}
		else
		{
			gAgent.setAlwaysRun();
//			gAgent.setRunning();
			report_to_nearby_chat(LLTrans::getString("AlwaysRunEnabled"));
		}

		// tell the simulator.
//		gAgent.sendWalkRun(gAgent.getAlwaysRun());

		// Update Movement Controls according to AlwaysRun mode
		LLFloaterMove::setAlwaysRunMode(gAgent.getAlwaysRun());

		return true;
	}
};

class LLWorldCheckAlwaysRun : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getAlwaysRun();
		return new_value;
	}
};

class LLWorldSetAway : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getAFK())
		{
			gAgent.clearAFK();
		}
		else
		{
			gAgent.setAFK();
		}
		return true;
	}
};
// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetAway : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getAFK();
		return new_value;
	}
};

class LLWorldSetDoNotDisturb : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.isDoNotDisturb())
		{
			gAgent.setDoNotDisturb(false);
		}
		else
		{
			gAgent.setDoNotDisturb(true);
			LLNotificationsUtil::add("DoNotDisturbModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetBusy : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.isDoNotDisturb();
		return new_value;
	}
};


class LLWorldSetAutorespond : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getAutorespond())
		{
			gAgent.clearAutorespond();
		}
		else
		{
			gAgent.setAutorespond();
			LLNotificationsUtil::add("AutorespondModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetAutorespond : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getAutorespond();
		return new_value;
	}
};


class LLWorldSetAutorespondNonFriends : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getAutorespondNonFriends())
		{
			gAgent.clearAutorespondNonFriends();
		}
		else
		{
			gAgent.setAutorespondNonFriends();
			LLNotificationsUtil::add("AutorespondNonFriendsModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetAutorespondNonFriends : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getAutorespondNonFriends();
		return new_value;
	}
};

// <FS:PP> FIRE-1245: Option to block/reject teleport offers
class LLWorldSetRejectTeleportOffers : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getRejectTeleportOffers())
		{
			gAgent.clearRejectTeleportOffers();
		}
		else
		{
			gAgent.setRejectTeleportOffers();
			LLNotificationsUtil::add("RejectTeleportOffersModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetRejectTeleportOffers : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getRejectTeleportOffers();
		return new_value;
	}
};
// </FS:PP> FIRE-1245: Option to block/reject teleport offers

// <FS:PP> FIRE-15233: Automatic friendship request refusal
class LLWorldSetRejectFriendshipRequests : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getRejectFriendshipRequests())
		{
			gAgent.clearRejectFriendshipRequests();
		}
		else
		{
			gAgent.setRejectFriendshipRequests();
			LLNotificationsUtil::add("RejectFriendshipRequestsModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetRejectFriendshipRequests : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getRejectFriendshipRequests();
		return new_value;
	}
};
// </FS:PP> FIRE-15233: Automatic friendship request refusal

// <FS:PP> Option to block/reject all group invites
class LLWorldSetRejectAllGroupInvites : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgent.getRejectAllGroupInvites())
		{
			gAgent.clearRejectAllGroupInvites();
		}
		else
		{
			gAgent.setRejectAllGroupInvites();
			LLNotificationsUtil::add("RejectAllGroupInvitesModeSet");
		}
		return true;
	}
};

// [SJ - FIRE-2177 - Making Autorespons a simple Check in the menu again for clarity]
class LLWorldGetRejectAllGroupInvites : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.getRejectAllGroupInvites();
		return new_value;
	}
};
// </FS:PP> Option to block/reject all group invites

class LLWorldCreateLandmark : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.4.5) | Added: RLVa-1.0.0
		if (gRlvHandler.hasBehaviour(RLV_BHVR_SHOWLOC))
			return true;
// [/RLVa:KB]

		// <FS:Ansariel> FIRE-817: Separate place details floater
		//LLFloaterSidePanelContainer::showPanel("places", LLSD().with("type", "create_landmark"));
		FSFloaterPlaceDetails::showPlaceDetails(LLSD().with("type", "create_landmark"));
		// </FS:Ansariel>

		return true;
	}
};

class LLWorldPlaceProfile : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2012-02-08 (RLVa-1.4.5) | Added: RLVa-1.4.5
		if (gRlvHandler.hasBehaviour(RLV_BHVR_SHOWLOC))
			return true;
// [/RLVa:KB]

		// <FS:Ansariel> FIRE-817: Separate place details floater
		//LLFloaterSidePanelContainer::showPanel("places", LLSD().with("type", "agent"));
		FSFloaterPlaceDetails::showPlaceDetails(LLSD().with("type", "agent"));
		// </FS:Ansariel>

		return true;
	}
};

// [RLVa:KB] - Checked: 2012-02-08 (RLVa-1.4.5) | Added: RLVa-1.4.5
bool enable_place_profile()
{
	return LLFloaterSidePanelContainer::canShowPanel("places", LLSD().with("type", "agent"));
}
// [/RLVa:KB]

void handle_script_info()
{
	LLUUID object_id;
	if (LLSelectMgr::getInstance()->getSelection()->getPrimaryObject())
	{
		object_id = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject()->mID;
		LL_INFOS() << "Reporting Script Info for object: " << object_id.asString() << LL_ENDL;
		FSLSLBridge::instance().viewerToLSL("getScriptInfo|" + object_id.asString());
	}
}

void handle_look_at_selection(const LLSD& param)
{
	const F32 PADDING_FACTOR = 1.75f;
	BOOL zoom = (param.asString() == "zoom");
	if (!LLSelectMgr::getInstance()->getSelection()->isEmpty())
	{
		gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);

		LLBBox selection_bbox = LLSelectMgr::getInstance()->getBBoxOfSelection();
		F32 angle_of_view = llmax(0.1f, LLViewerCamera::getInstance()->getAspect() > 1.f ? LLViewerCamera::getInstance()->getView() * LLViewerCamera::getInstance()->getAspect() : LLViewerCamera::getInstance()->getView());
		F32 distance = selection_bbox.getExtentLocal().magVec() * PADDING_FACTOR / atan(angle_of_view);

		LLVector3 obj_to_cam = LLViewerCamera::getInstance()->getOrigin() - selection_bbox.getCenterAgent();
		obj_to_cam.normVec();

		LLUUID object_id;
		if (LLSelectMgr::getInstance()->getSelection()->getPrimaryObject())
		{
			object_id = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject()->mID;
		}
		if (zoom)
		{
			// Make sure we are not increasing the distance between the camera and object
			LLVector3d orig_distance = gAgentCamera.getCameraPositionGlobal() - LLSelectMgr::getInstance()->getSelectionCenterGlobal();
			distance = llmin(distance, (F32) orig_distance.length());
				
			gAgentCamera.setCameraPosAndFocusGlobal(LLSelectMgr::getInstance()->getSelectionCenterGlobal() + LLVector3d(obj_to_cam * distance), 
										LLSelectMgr::getInstance()->getSelectionCenterGlobal(), 
										object_id );
			
		}
		else
		{
			gAgentCamera.setFocusGlobal( LLSelectMgr::getInstance()->getSelectionCenterGlobal(), object_id );
		}	
	}
}

// <FS:Ansariel> Option to try via exact position
//void handle_zoom_to_object(LLUUID object_id)
void handle_zoom_to_object(LLUUID object_id, const LLVector3d& object_pos)
// </FS:Ansariel> Option to try via exact position
{
	// <FS:Zi> Fix camera zoom to look at the avatar's face from the front
	// const F32 PADDING_FACTOR = 2.f;
	// </FS:Zi>

	LLViewerObject* object = gObjectList.findObject(object_id);

	if (object)
	{
		gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);

		// <FS:Zi> Fix camera zoom to look at the avatar's face from the front
		// LLBBox bbox = object->getBoundingBoxAgent() ;
		// F32 angle_of_view = llmax(0.1f, LLViewerCamera::getInstance()->getAspect() > 1.f ? LLViewerCamera::getInstance()->getView() * LLViewerCamera::getInstance()->getAspect() : LLViewerCamera::getInstance()->getView());
		// F32 distance = bbox.getExtentLocal().magVec() * PADDING_FACTOR / atan(angle_of_view);

		// LLVector3 obj_to_cam = LLViewerCamera::getInstance()->getOrigin() - bbox.getCenterAgent();
		// obj_to_cam.normVec();


		//	LLVector3d object_center_global = gAgent.getPosGlobalFromAgent(bbox.getCenterAgent());

		// 	gAgentCamera.setCameraPosAndFocusGlobal(object_center_global + LLVector3d(obj_to_cam * distance), 
		// 									object_center_global, 

		LLVector3d object_center_global=object->getPositionGlobal();

		float eye_distance=gSavedSettings.getF32("CameraZoomDistance");
		float eye_z_offset=gSavedSettings.getF32("CameraZoomEyeZOffset");
		LLVector3d focus_z_offset=LLVector3d(0.0f,0.0f,gSavedSettings.getF32("CameraZoomFocusZOffset"));

		LLVector3d eye_offset(eye_distance,0.0f,eye_z_offset);
		eye_offset=eye_offset*object->getRotationRegion();

		gAgentCamera.setCameraPosAndFocusGlobal(object_center_global+eye_offset, 
										object_center_global+focus_z_offset, 
		// </FS:Zi>
											object_id );
	}
	// <FS:Ansariel> Option to try via exact position
	else if (object_pos != LLVector3d(-1.f, -1.f, -1.f))
	{
		LLVector3d obj_to_cam = object_pos - gAgent.getPositionGlobal();
		obj_to_cam.normVec();
		obj_to_cam = obj_to_cam * -4.f;
		obj_to_cam.mdV[VZ] += 0.5;

		gAgentCamera.changeCameraToThirdPerson();
		gAgentCamera.unlockView();
		gAgentCamera.setCameraPosAndFocusGlobal(object_pos + obj_to_cam, object_pos, object_id);
	}
	// </FS:Ansariel> Option to try via exact position
}

class LLAvatarInviteToGroup : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
//		if(avatar)
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) )
// [/RLVa:KB]
		{
			LLAvatarActions::inviteToGroup(avatar->getID());
		}
		return true;
	}
};

class LLAvatarAddFriend : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
//		if(avatar && !LLAvatarActions::isFriend(avatar->getID()))
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar && !LLAvatarActions::isFriend(avatar->getID())) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) )
// [/RLVa:KB]
		{
			request_friendship(avatar->getID());
		}
		return true;
	}
};


class LLAvatarToggleMyProfile : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFloater* instance = LLAvatarActions::getProfileFloater(gAgent.getID());
		if (LLFloater::isMinimized(instance))
		{
			instance->setMinimized(FALSE);
			instance->setFocus(TRUE);
		}
		else if (!LLFloater::isShown(instance))
		{
			LLAvatarActions::showProfile(gAgent.getID());
		}
		else if (!instance->hasFocus() && !instance->getIsChrome())
		{
			instance->setFocus(TRUE);
		}
		else
		{
			instance->closeFloater();
		}
		return true;
	}
};

class LLAvatarResetSkeleton: public view_listener_t
{
    bool handleEvent(const LLSD& userdata)
    {
        // <FS:Ansariel> Fix reset skeleton not working
		//LLVOAvatar* avatar = NULL;
        //LLViewerObject *obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
        //if (obj)
        //{
        //    avatar = obj->getAvatar();
        //}
        LLVOAvatar* avatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
        // </FS:Ansariel>
		if(avatar)
        {
            avatar->resetSkeleton(false);
        }
        return true;
    }
};

class LLAvatarEnableResetSkeleton: public view_listener_t
{
    bool handleEvent(const LLSD& userdata)
    {
        LLViewerObject *obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
        if (obj && obj->getAvatar())
        {
            return true;
        }
        return false;
    }
};


class LLAvatarResetSkeletonAndAnimations : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (avatar)
		{
			avatar->resetSkeleton(true);
		}
		return true;
	}
};

class LLAvatarAddContact : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
//		if(avatar)
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) )
// [/RLVa:KB]
		{
			create_inventory_callingcard(avatar->getID());
		}
		return true;
	}
};

bool complete_give_money(const LLSD& notification, const LLSD& response, LLObjectSelectionHandle selection)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (option == 0)
	{
		gAgent.setDoNotDisturb(false);
	}

	LLViewerObject* objectp = selection->getPrimaryObject();

	// Show avatar's name if paying attachment
	if (objectp && objectp->isAttachment())
	{
		while (objectp && !objectp->isAvatar())
		{
			objectp = (LLViewerObject*)objectp->getParent();
		}
	}

	if (objectp)
	{
		if (objectp->isAvatar())
		{
			const bool is_group = false;
			LLFloaterPayUtil::payDirectly(&give_money,
									  objectp->getID(),
									  is_group);
		}
		else
		{
			LLFloaterPayUtil::payViaObject(&give_money, selection);
		}
	}
	return false;
}

void handle_give_money_dialog()
{
	LLNotification::Params params("DoNotDisturbModePay");
	params.functor.function(boost::bind(complete_give_money, _1, _2, LLSelectMgr::getInstance()->getSelection()));

	if (gAgent.isDoNotDisturb())
	{
		// warn users of being in do not disturb mode during a transaction
		LLNotifications::instance().add(params);
	}
	else
	{
		LLNotifications::instance().forceResponse(params, 1);
	}
}

bool enable_pay_avatar()
{
	LLViewerObject* obj = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	LLVOAvatar* avatar = find_avatar_from_object(obj);
//	return (avatar != NULL);
// [RLVa:KB] - Checked: RLVa-1.2.1
	return (avatar != NULL) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID()));
// [/RLVa:KB]
}

bool enable_pay_object()
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if( object )
	{
		LLViewerObject *parent = (LLViewerObject *)object->getParent();
		if((object->flagTakesMoney()) || (parent && parent->flagTakesMoney()))
		{
			return true;
		}
	}
	return false;
}

bool enable_object_stand_up()
{
	// 'Object Stand Up' menu item is enabled when agent is sitting on selection
//	return sitting_on_selection();
// [RLVa:KB] - Checked: 2010-07-24 (RLVa-1.2.0g) | Added: RLVa-1.2.0g
	return sitting_on_selection() && ( (!rlv_handler_t::isEnabled()) || (RlvActions::canStand()) );
// [/RLVa:KB]
}

bool enable_object_sit(LLUICtrl* ctrl)
{
	// 'Object Sit' menu item is enabled when agent is not sitting on selection
	bool sitting_on_sel = sitting_on_selection();
	if (!sitting_on_sel)
	{
		std::string item_name = ctrl->getName();

		// init default labels
		init_default_item_label(item_name);

		// Update label
		LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
		if (node && node->mValid && !node->mSitName.empty())
		{
			gMenuHolder->childSetValue(item_name, node->mSitName);
		}
		else
		{
			gMenuHolder->childSetValue(item_name, get_default_item_label(item_name));
		}
	}

// [RLVa:KB] - Checked: 2010-04-01 (RLVa-1.2.0c) | Modified: RLVa-1.2.0c
		// RELEASE-RLVA: [SL-2.2.0] Make this match what happens in handle_object_sit_or_stand()
		if (rlv_handler_t::isEnabled())
		{
			const LLPickInfo& pick = LLToolPie::getInstance()->getPick();
			if (pick.mObjectID.notNull())
				sitting_on_sel = !RlvActions::canSit(pick.getObject(), pick.mObjectOffset);
		}
// [/RLVa:KB]

	return !sitting_on_sel && is_object_sittable();
}

void dump_select_mgr(void*)
{
	LLSelectMgr::getInstance()->dump();
}

void dump_inventory(void*)
{
	gInventory.dumpInventory();
}


void handle_dump_followcam(void*)
{
	LLFollowCamMgr::dump();
}

void handle_viewer_enable_message_log(void*)
{
	gMessageSystem->startLogging();
}

void handle_viewer_disable_message_log(void*)
{
	gMessageSystem->stopLogging();
}

void handle_customize_avatar()
{
	// <FS:Ansariel> FIRE-19614: Make CTRL-O toggle the appearance floater
	LLFloater* floater = LLFloaterReg::findInstance("appearance");
	if (floater && floater->isMinimized())
	{
		floater->setMinimized(FALSE);
	}
	else if (LLFloater::isShown(floater))
	{
		LLFloaterReg::hideInstance("appearance");
	}
	else
	// </FS:Ansariel>
	LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "my_outfits"));
}

void handle_edit_outfit()
{
	LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "edit_outfit"));
}

void handle_edit_shape()
{
	LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "edit_shape"));
}

void handle_hover_height()
{
	LLFloaterReg::showInstance("edit_hover_height");
}

void handle_edit_physics()
{
	LLFloaterSidePanelContainer::showPanel("appearance", LLSD().with("type", "edit_physics"));
}

void handle_report_abuse()
{
	// Prevent menu from appearing in screen shot.
	gMenuHolder->hideMenus();
	LLFloaterReporter::showFromMenu(COMPLAINT_REPORT);
}

void handle_buy_currency()
{
	LLBuyCurrencyHTML::openCurrencyFloater();
}

void handle_recreate_lsl_bridge()
{
	FSLSLBridge::instance().recreateBridge();
}

class LLFloaterVisible : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string floater_name = userdata.asString();
		bool new_value = false;
		{
			new_value = LLFloaterReg::instanceVisible(floater_name);
		}
		return new_value;
	}
};

class LLShowHelp : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string help_topic = userdata.asString();
#ifdef OPENSIM
		if (help_topic.find("grid_") != std::string::npos)
		{
			help_topic.erase(0,5);
			
			std::string url;
			LLSD grid_info;
			LLGridManager::getInstance()->getGridData(grid_info);
			if (grid_info.has(help_topic))
			{
				url = grid_info[help_topic].asString();
			}
			
			if(!url.empty())
			{
				LLWeb::loadURLInternal(url);
			}
			LL_DEBUGS() << "grid_help " <<  help_topic << " url " << url << LL_ENDL;

			return true;
		}
#endif // OPENSIM
		LLViewerHelp* vhelp = LLViewerHelp::getInstance();
		vhelp->showTopic(help_topic);
		
		return true;
	}
};

// <AW: OpenSim>
bool update_grid_help()
{
// <FS:AW  grid management>
	if (!gMenuHolder) //defend crash on shutdown
	{
		return false;
	}
// </FS:AW  grid management>

	bool needs_seperator = false;

#ifdef OPENSIM // <FS:AW optional opensim support>
	LLSD grid_info;
	LLGridManager::getInstance()->getGridData(grid_info);
	std::string grid_label = LLGridManager::getInstance()->getGridLabel();
	bool is_opensim = LLGridManager::getInstance()->isInOpenSim();
	if (is_opensim && grid_info.has("help"))
	{
		needs_seperator = true;
		gMenuHolder->childSetVisible("current_grid_help",true);
		gMenuHolder->childSetLabelArg("current_grid_help", "[CURRENT_GRID]", grid_label);
		gMenuHolder->childSetVisible("current_grid_help_login",true);
		gMenuHolder->childSetLabelArg("current_grid_help_login", "[CURRENT_GRID]", grid_label);
	}
	else
#endif // OPENSIM // <FS:AW optional opensim support>
	{
		gMenuHolder->childSetVisible("current_grid_help",false);
		gMenuHolder->childSetVisible("current_grid_help_login",false);
	}
#ifdef OPENSIM // <FS:AW optional opensim support>
	if (is_opensim && grid_info.has("about"))
	{
		needs_seperator = true;
		gMenuHolder->childSetVisible("current_grid_about",true);
		gMenuHolder->childSetLabelArg("current_grid_about", "[CURRENT_GRID]", grid_label);
		gMenuHolder->childSetVisible("current_grid_about_login",true);
		gMenuHolder->childSetLabelArg("current_grid_about_login", "[CURRENT_GRID]", grid_label);
	}
	else
#endif // OPENSIM // <FS:AW optional opensim support>
	{
		gMenuHolder->childSetVisible("current_grid_about",false);
		gMenuHolder->childSetVisible("current_grid_about_login",false);
	}
	//FIXME: this does nothing
	gMenuHolder->childSetVisible("grid_help_seperator",needs_seperator);
	gMenuHolder->childSetVisible("grid_help_seperator_login",needs_seperator);

// <FS:AW  opensim destinations and avatar picker>
#ifdef OPENSIM // <FS:AW optional opensim support>
	if (is_opensim)
	{
		if (!LLLoginInstance::getInstance()->hasResponse("destination_guide_url") 
		||LLLoginInstance::getInstance()->getResponse("destination_guide_url").asString().empty()
		)
		{
			gMenuHolder->childSetVisible("Avatar Picker", false);
		}
	
		if (!LLLoginInstance::getInstance()->hasResponse("avatar_picker_url") 
		||LLLoginInstance::getInstance()->getResponse("avatar_picker_url").asString().empty()
		)
		{
			gMenuHolder->childSetVisible("Destinations", false);
		}
	}
#endif // OPENSIM // <FS:AW optional opensim support>
// </FS:AW  opensim destinations and avatar picker>

	return true;
}
// </AW: OpenSim>

class LLToggleHelp : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFloater* help_browser = (LLFloaterReg::findInstance("help_browser"));
		if (help_browser && help_browser->isInVisibleChain())
		{
			help_browser->closeFloater();
		}
		else
		{
			std::string help_topic = userdata.asString();
			LLViewerHelp* vhelp = LLViewerHelp::getInstance();
			vhelp->showTopic(help_topic);
		}
		return true;
	}
};

class LLToggleSpeak : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVoiceClient::getInstance()->toggleUserPTTState();
		return true;
	}
};

bool callback_show_url(const LLSD& notification, const LLSD& response)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (0 == option)
	{
		LLWeb::loadURL(notification["payload"]["url"].asString());
	}
	return false;
}

class LLPromptShowURL : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string param = userdata.asString();
		std::string::size_type offset = param.find(",");
		if (offset != param.npos)
		{
			std::string alert = param.substr(0, offset);
			std::string url = param.substr(offset+1);

			if (LLWeb::useExternalBrowser(url))
			{ 
				// <FS:Ansariel> FS-1951: LLWeb::loadURL() will spawn the WebLaunchExternalTarget
				//               confirmation if opening with an external browser
    			//LLSD payload;
    			//payload["url"] = url;
    			//LLNotificationsUtil::add(alert, LLSD(), payload, callback_show_url);
				if (alert == "WebLaunchExternalTarget")
				{
					LLWeb::loadURL(url);
				}
				else
				{
					LLSD payload;
					payload["url"] = url;
					LLNotificationsUtil::add(alert, LLSD(), payload, callback_show_url);
				}
				// </FS:Ansariel>
			}
			else
			{
		        LLWeb::loadURL(url);
			}
		}
		else
		{
			LL_INFOS() << "PromptShowURL invalid parameters! Expecting \"ALERT,URL\"." << LL_ENDL;
		}
		return true;
	}
};

bool callback_show_file(const LLSD& notification, const LLSD& response)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (0 == option)
	{
		LLWeb::loadURL(notification["payload"]["url"]);
	}
	return false;
}

class LLPromptShowFile : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string param = userdata.asString();
		std::string::size_type offset = param.find(",");
		if (offset != param.npos)
		{
			std::string alert = param.substr(0, offset);
			std::string file = param.substr(offset+1);

			LLSD payload;
			payload["url"] = file;
			LLNotificationsUtil::add(alert, LLSD(), payload, callback_show_file);
		}
		else
		{
			LL_INFOS() << "PromptShowFile invalid parameters! Expecting \"ALERT,FILE\"." << LL_ENDL;
		}
		return true;
	}
};

class LLShowAgentProfile : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLUUID agent_id;
		if (userdata.asString() == "agent")
		{
			agent_id = gAgent.getID();
		}
		else if (userdata.asString() == "hit object")
		{
			LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
			if (objectp)
			{
				agent_id = objectp->getID();
			}
		}
		else
		{
			agent_id = userdata.asUUID();
		}

		LLVOAvatar* avatar = find_avatar_from_object(agent_id);
//		if (avatar)
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar) && ((RlvActions::canShowName(RlvActions::SNC_DEFAULT, agent_id)) || (gAgent.getID() == agent_id)) )
// [/RLVa:KB]
		{
			LLAvatarActions::showProfile(avatar->getID());
		}
		return true;
	}
};

class LLToggleAgentProfile : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLUUID agent_id;
		if (userdata.asString() == "agent")
		{
			agent_id = gAgent.getID();
		}
		else if (userdata.asString() == "hit object")
		{
			LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
			if (objectp)
			{
				agent_id = objectp->getID();
			}
		}
		else
		{
			agent_id = userdata.asUUID();
		}

		LLVOAvatar* avatar = find_avatar_from_object(agent_id);
		if (avatar)
		{
			if (!LLAvatarActions::profileVisible(avatar->getID()))
			{
				LLAvatarActions::showProfile(avatar->getID());
			}
			else
			{
				LLAvatarActions::hideProfile(avatar->getID());
			}
		}
		return true;
	}
};

class LLLandEdit : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (gAgentCamera.getFocusOnAvatar() && gSavedSettings.getBOOL("EditCameraMovement") )
		{
			// zoom in if we're looking at the avatar
			gAgentCamera.setFocusOnAvatar(FALSE, ANIMATE);
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());

			gAgentCamera.cameraOrbitOver( F_PI * 0.25f );
			gViewerWindow->moveCursorToCenter();
		}
		else if ( gSavedSettings.getBOOL("EditCameraMovement") )
		{
			gAgentCamera.setFocusGlobal(LLToolPie::getInstance()->getPick());
			gViewerWindow->moveCursorToCenter();
		}


		LLViewerParcelMgr::getInstance()->selectParcelAt( LLToolPie::getInstance()->getPick().mPosGlobal );

		LLFloaterReg::showInstance("build");

		// Switch to land edit toolset
		LLToolMgr::getInstance()->getCurrentToolset()->selectTool( LLToolSelectLand::getInstance() );
		return true;
	}
};

class LLMuteParticle : public view_listener_t
{
	// <FS:Ansariel> Blocklist sometimes shows "(waiting)" as avatar name when blocking particle owners
	void onAvatarNameCache(const LLUUID& av_id, const LLAvatarName& av_name)
	{
		LLMute mute(av_id, av_name.getUserName(), LLMute::AGENT);
		if (LLMuteList::getInstance()->isMuted(mute.mID))
		{
			LLMuteList::getInstance()->remove(mute);
		}
		else
		{
			LLMuteList::getInstance()->add(mute);
			LLPanelBlockedList::showPanelAndSelect(mute.mID);
		}
	}
	// </FS:Ansariel>

	bool handleEvent(const LLSD& userdata)
	{
		LLUUID id = LLToolPie::getInstance()->getPick().mParticleOwnerID;
		
		if (id.notNull())
		{
			// <FS:Ansariel> Blocklist sometimes shows "(waiting)" as avatar name when blocking particle owners
			//LLAvatarName av_name;
			//LLAvatarNameCache::get(id, &av_name);

			//LLMute mute(id, av_name.getUserName(), LLMute::AGENT);
			//if (LLMuteList::getInstance()->isMuted(mute.mID))
			//{
			//	LLMuteList::getInstance()->remove(mute);
			//}
			//else
			//{
			//	LLMuteList::getInstance()->add(mute);
			//	LLPanelBlockedList::showPanelAndSelect(mute.mID);
			//}
			LLAvatarNameCache::get(id, boost::bind(&LLMuteParticle::onAvatarNameCache, this, _1, _2));
			// </FS:Ansariel>
		}

		return true;
	}
};

class LLWorldEnableBuyLand : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLViewerParcelMgr::getInstance()->canAgentBuyParcel(
								LLViewerParcelMgr::getInstance()->selectionEmpty()
									? LLViewerParcelMgr::getInstance()->getAgentParcel()
									: LLViewerParcelMgr::getInstance()->getParcelSelection()->getParcel(),
								false);
		return new_value;
	}
};

BOOL enable_buy_land(void*)
{
	return LLViewerParcelMgr::getInstance()->canAgentBuyParcel(
				LLViewerParcelMgr::getInstance()->getParcelSelection()->getParcel(), false);
}

void handle_buy_land()
{
	LLViewerParcelMgr* vpm = LLViewerParcelMgr::getInstance();
	if (vpm->selectionEmpty())
	{
		vpm->selectParcelAt(gAgent.getPositionGlobal());
	}
	vpm->startBuyLand();
}

class LLObjectAttachToAvatar : public view_listener_t
{
public:
	LLObjectAttachToAvatar(bool replace) : mReplace(replace) {}
	static void setObjectSelection(LLObjectSelectionHandle selection) { sObjectSelection = selection; }

private:
	bool handleEvent(const LLSD& userdata)
	{
		setObjectSelection(LLSelectMgr::getInstance()->getSelection());
		LLViewerObject* selectedObject = sObjectSelection->getFirstRootObject();
		if (selectedObject)
		{
			S32 index = userdata.asInteger();
			LLViewerJointAttachment* attachment_point = NULL;
			if (index > 0)
				attachment_point = get_if_there(gAgentAvatarp->mAttachmentPoints, index, (LLViewerJointAttachment*)NULL);

// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
			// RELEASE-RLVa: [SL-2.2.0] If 'index != 0' then the object will be "add attached" [see LLSelectMgr::sendAttach()]
			if ( (rlv_handler_t::isEnabled()) &&
				 ( ((!index) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_ANY))) ||		    // Can't wear on default
				   ((index) && ((RLV_WEAR_ADD & gRlvAttachmentLocks.canAttach(attachment_point)) == 0)) ||	// or non-attachable attachpt
				   (gRlvHandler.hasBehaviour(RLV_BHVR_REZ)) ) )											    // Attach on object == "Take"
			{
				setObjectSelection(NULL); // Clear the selection or it'll get stuck
				return true;
			}
// [/RLVa:KB]

			confirmReplaceAttachment(0, attachment_point);
		}
		return true;
	}

	static void onNearAttachObject(BOOL success, void *user_data);
	void confirmReplaceAttachment(S32 option, LLViewerJointAttachment* attachment_point);

	struct CallbackData
	{
		CallbackData(LLViewerJointAttachment* point, bool replace) : mAttachmentPoint(point), mReplace(replace) {}

		LLViewerJointAttachment*	mAttachmentPoint;
		bool						mReplace;
	};

protected:
	static LLObjectSelectionHandle sObjectSelection;
	bool mReplace;
};

LLObjectSelectionHandle LLObjectAttachToAvatar::sObjectSelection;

// static
void LLObjectAttachToAvatar::onNearAttachObject(BOOL success, void *user_data)
{
	if (!user_data) return;
	CallbackData* cb_data = static_cast<CallbackData*>(user_data);

	if (success)
	{
		const LLViewerJointAttachment *attachment = cb_data->mAttachmentPoint;
		
		U8 attachment_id = 0;
		if (attachment)
		{
			for (LLVOAvatar::attachment_map_t::const_iterator iter = gAgentAvatarp->mAttachmentPoints.begin();
				 iter != gAgentAvatarp->mAttachmentPoints.end(); ++iter)
			{
				if (iter->second == attachment)
				{
					attachment_id = iter->first;
					break;
				}
			}
		}
		else
		{
			// interpret 0 as "default location"
			attachment_id = 0;
		}
		LLSelectMgr::getInstance()->sendAttach(attachment_id, cb_data->mReplace);
	}		
	LLObjectAttachToAvatar::setObjectSelection(NULL);

	delete cb_data;
}

// static
void LLObjectAttachToAvatar::confirmReplaceAttachment(S32 option, LLViewerJointAttachment* attachment_point)
{
	if (option == 0/*YES*/)
	{
		LLViewerObject* selectedObject = LLSelectMgr::getInstance()->getSelection()->getFirstRootObject();
		if (selectedObject)
		{
			const F32 MIN_STOP_DISTANCE = 1.f;	// meters
			const F32 ARM_LENGTH = 0.5f;		// meters
			const F32 SCALE_FUDGE = 1.5f;

			F32 stop_distance = SCALE_FUDGE * selectedObject->getMaxScale() + ARM_LENGTH;
			if (stop_distance < MIN_STOP_DISTANCE)
			{
				stop_distance = MIN_STOP_DISTANCE;
			}

			LLVector3 walkToSpot = selectedObject->getPositionAgent();
			
			// make sure we stop in front of the object
			LLVector3 delta = walkToSpot - gAgent.getPositionAgent();
			delta.normVec();
			delta = delta * 0.5f;
			walkToSpot -= delta;

			// The callback will be called even if avatar fails to get close enough to the object, so we won't get a memory leak.
			CallbackData* user_data = new CallbackData(attachment_point, mReplace);
			gAgent.startAutoPilotGlobal(gAgent.getPosGlobalFromAgent(walkToSpot), "Attach", NULL, onNearAttachObject, user_data, stop_distance);
			gAgentCamera.clearFocusObject();
		}
	}
}

void callback_attachment_drop(const LLSD& notification, const LLSD& response)
{
	// Ensure user confirmed the drop
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if (option != 0) return;

	// Called when the user clicked on an object attached to them
	// and selected "Drop".
	LLUUID object_id = notification["payload"]["object_id"].asUUID();
	LLViewerObject *object = gObjectList.findObject(object_id);
	
	if (!object)
	{
		LL_WARNS() << "handle_drop_attachment() - no object to drop" << LL_ENDL;
		return;
	}

	LLViewerObject *parent = (LLViewerObject*)object->getParent();
	while (parent)
	{
		if(parent->isAvatar())
		{
			break;
		}
		object = parent;
		parent = (LLViewerObject*)parent->getParent();
	}

	if (!object)
	{
		LL_WARNS() << "handle_detach() - no object to detach" << LL_ENDL;
		return;
	}

	if (object->isAvatar())
	{
		LL_WARNS() << "Trying to detach avatar from avatar." << LL_ENDL;
		return;
	}
	
	// reselect the object
	LLSelectMgr::getInstance()->selectObjectAndFamily(object);

	LLSelectMgr::getInstance()->sendDropAttachment();

	return;
}

class LLAttachmentDrop : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-03-15 (RLVa-1.2.0e) | Modified: RLVa-1.0.5
		if (rlv_handler_t::isEnabled())
		{
			if (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE))
			{
				// NOTE: copy/paste of the code in enable_detach()
				LLObjectSelectionHandle hSelect = LLSelectMgr::getInstance()->getSelection();
				RlvSelectHasLockedAttach f;
				if ( (hSelect->isAttachment()) && (hSelect->getFirstRootNode(&f, FALSE) != NULL) )
					return true;
			}
			if (gRlvHandler.hasBehaviour(RLV_BHVR_REZ))
			{
				return true;
			}
		}
// [/RLVa:KB]

		LLSD payload;
		LLViewerObject *object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();

		if (object) 
		{
			payload["object_id"] = object->getID();
		}
		else
		{
			LL_WARNS() << "Drop object not found" << LL_ENDL;
			return true;
		}

		LLNotificationsUtil::add("AttachmentDrop", LLSD(), payload, &callback_attachment_drop);
		return true;
	}
};

// called from avatar pie menu
class LLAttachmentDetachFromPoint : public view_listener_t
{
	bool handleEvent(const LLSD& user_data)
	{
		uuid_vec_t ids_to_remove;
		const LLViewerJointAttachment *attachment = get_if_there(gAgentAvatarp->mAttachmentPoints, user_data.asInteger(), (LLViewerJointAttachment*)NULL);
//		if (attachment->getNumObjects() > 0)
// [RLVa:KB] - Checked: 2010-03-04 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
		if ( (attachment->getNumObjects() > 0) && ((!rlv_handler_t::isEnabled()) || (gRlvAttachmentLocks.canDetach(attachment))) )
// [/RLVa:KB]
		{
			for (LLViewerJointAttachment::attachedobjs_vec_t::const_iterator iter = attachment->mAttachedObjects.begin();
				 iter != attachment->mAttachedObjects.end();
				 iter++)
			{
				LLViewerObject *attached_object = (*iter);
// [RLVa:KB] - Checked: 2010-03-04 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
				if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.isLockedAttachment(attached_object)) )
					continue;
				ids_to_remove.push_back(attached_object->getAttachmentItemID());
// [/RLVa:KB]
			}
        }
		if (!ids_to_remove.empty())
		{
			LLAppearanceMgr::instance().removeItemsFromAvatar(ids_to_remove);
		}
		return true;
	}
};

static bool onEnableAttachmentLabel(LLUICtrl* ctrl, const LLSD& data)
{
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
	// RELEASE-RLVa: [SL-2.2.0] When attaching to a specific point the object will be "add attached" [see LLSelectMgr::sendAttach()]
	bool fRlvEnable = true;
// [/RLVa:KB]
	std::string label;
	LLMenuItemGL* menu = dynamic_cast<LLMenuItemGL*>(ctrl);
	if (menu)
	{
		const LLViewerJointAttachment *attachment = get_if_there(gAgentAvatarp->mAttachmentPoints, data["index"].asInteger(), (LLViewerJointAttachment*)NULL);
		if (attachment)
		{
			label = data["label"].asString();
			for (LLViewerJointAttachment::attachedobjs_vec_t::const_iterator attachment_iter = attachment->mAttachedObjects.begin();
				 attachment_iter != attachment->mAttachedObjects.end();
				 ++attachment_iter)
			{
				const LLViewerObject* attached_object = (*attachment_iter);
				if (attached_object)
				{
					LLViewerInventoryItem* itemp = gInventory.getItem(attached_object->getAttachmentItemID());
					// <FS:Ansariel> Hide bridge from attach to HUD menus
					//if (itemp)
					if (itemp && !(FSLSLBridge::instance().getBridge() && FSLSLBridge::instance().getBridge()->getUUID() == itemp->getUUID() && data["index"].asInteger() == FS_BRIDGE_POINT))
					// </FS:Ansariel>
					{
						label += std::string(" (") + itemp->getName() + std::string(")");
						break;
					}
				}
			}
		}

// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
		if (rlv_handler_t::isEnabled())
			fRlvEnable = (!gRlvAttachmentLocks.isLockedAttachmentPoint(attachment, RLV_LOCK_ADD));
// [/RLVa:KB]

		menu->setLabel(label);
	}
//	return true;
// [RLVa:KB] - Checked: 2010-02-27 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
	return fRlvEnable;
// [/RLVa:KB]
}

class LLAttachmentDetach : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// Called when the user clicked on an object attached to them
		// and selected "Detach".
		LLViewerObject *object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (!object)
		{
			LL_WARNS() << "handle_detach() - no object to detach" << LL_ENDL;
			return true;
		}

		LLViewerObject *parent = (LLViewerObject*)object->getParent();
		while (parent)
		{
			if(parent->isAvatar())
			{
				break;
			}
			object = parent;
			parent = (LLViewerObject*)parent->getParent();
		}

		if (!object)
		{
			LL_WARNS() << "handle_detach() - no object to detach" << LL_ENDL;
			return true;
		}

		if (object->isAvatar())
		{
			LL_WARNS() << "Trying to detach avatar from avatar." << LL_ENDL;
			return true;
		}

// [RLVa:KB] - Checked: 2010-03-15 (RLVa-1.2.0a) | Modified: RLVa-1.0.5
		// NOTE: copy/paste of the code in enable_detach()
		if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) )
		{
			LLObjectSelectionHandle hSelect = LLSelectMgr::getInstance()->getSelection();
			RlvSelectHasLockedAttach f;
			if ( (hSelect->isAttachment()) && (hSelect->getFirstRootNode(&f, FALSE) != NULL) )
				return true;
		}
// [/RLVa:KB]

		LLAppearanceMgr::instance().removeItemFromAvatar(object->getAttachmentItemID());

		return true;
	}
};

//Adding an observer for a Jira 2422 and needs to be a fetch observer
//for Jira 3119
class LLWornItemFetchedObserver : public LLInventoryFetchItemsObserver
{
public:
	LLWornItemFetchedObserver(const LLUUID& worn_item_id) :
		LLInventoryFetchItemsObserver(worn_item_id)
	{}
	virtual ~LLWornItemFetchedObserver() {}

protected:
	virtual void done()
	{
		gMenuAttachmentSelf->buildDrawLabels();
		gInventory.removeObserver(this);
		delete this;
	}
};

// You can only drop items on parcels where you can build.
class LLAttachmentEnableDrop : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		BOOL can_build   = gAgent.isGodlike() || (LLViewerParcelMgr::getInstance()->allowAgentBuild());

		//Add an inventory observer to only allow dropping the newly attached item
		//once it exists in your inventory.  Look at Jira 2422.
		//-jwolk

		// A bug occurs when you wear/drop an item before it actively is added to your inventory
		// if this is the case (you're on a slow sim, etc.) a copy of the object,
		// well, a newly created object with the same properties, is placed
		// in your inventory.  Therefore, we disable the drop option until the
		// item is in your inventory

		LLViewerObject*              object         = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		LLViewerJointAttachment*     attachment     = NULL;
		LLInventoryItem*             item           = NULL;

		// Do not enable drop if all faces of object are not enabled
		if (object && LLSelectMgr::getInstance()->getSelection()->contains(object,SELECT_ALL_TES ))
		{
    		S32 attachmentID  = ATTACHMENT_ID_FROM_STATE(object->getAttachmentState());
			attachment = get_if_there(gAgentAvatarp->mAttachmentPoints, attachmentID, (LLViewerJointAttachment*)NULL);

			if (attachment)
			{
				for (LLViewerJointAttachment::attachedobjs_vec_t::iterator attachment_iter = attachment->mAttachedObjects.begin();
					 attachment_iter != attachment->mAttachedObjects.end();
					 ++attachment_iter)
				{
					// make sure item is in your inventory (it could be a delayed attach message being sent from the sim)
					// so check to see if the item is in the inventory already
					item = gInventory.getItem((*attachment_iter)->getAttachmentItemID());
					if (!item)
					{
						// Item does not exist, make an observer to enable the pie menu 
						// when the item finishes fetching worst case scenario 
						// if a fetch is already out there (being sent from a slow sim)
						// we refetch and there are 2 fetches
						LLWornItemFetchedObserver* worn_item_fetched = new LLWornItemFetchedObserver((*attachment_iter)->getAttachmentItemID());		
						worn_item_fetched->startFetch();
						gInventory.addObserver(worn_item_fetched);
					}
				}
			}
		}
		
		//now check to make sure that the item is actually in the inventory before we enable dropping it
//		bool new_value = enable_detach() && can_build && item;
// [RLVa:KB] - Checked: 2010-03-24 (RLVa-1.0.0b) | Modified: RLVa-1.0.0b
		bool new_value = enable_detach() && can_build && item && (!gRlvHandler.hasBehaviour(RLV_BHVR_REZ));
// [/RLVa:KB]

		return new_value;
	}
};

BOOL enable_detach(const LLSD&)
{
	LLViewerObject* object = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	
	// Only enable detach if all faces of object are selected
	if (!object ||
		!object->isAttachment() ||
		!LLSelectMgr::getInstance()->getSelection()->contains(object,SELECT_ALL_TES ))
	{
		return FALSE;
	}

	// Find the avatar who owns this attachment
	LLViewerObject* avatar = object;
	while (avatar)
	{
		// ...if it's you, good to detach
		if (avatar->getID() == gAgent.getID())
		{
// [RLVa:KB] - Checked: 2010-03-15 (RLVa-1.2.0a) | Modified: RLVa-1.0.5
			// NOTE: this code is reused as-is in LLAttachmentDetach::handleEvent() and LLAttachmentDrop::handleEvent()
			//       so any changes here should be reflected there as well

			// RELEASE-RLVa: [SL-2.2.0] LLSelectMgr::sendDetach() and LLSelectMgr::sendDropAttachment() call sendListToRegions with
			//                          SEND_ONLY_ROOTS so we only need to examine the roots which saves us time
			if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) )
			{
				LLObjectSelectionHandle hSelect = LLSelectMgr::getInstance()->getSelection();
				RlvSelectHasLockedAttach f;
				if ( (hSelect->isAttachment()) && (hSelect->getFirstRootNode(&f, FALSE) != NULL) )
					return FALSE;
			}
// [/RLVa:KB]
			return TRUE;
		}

		avatar = (LLViewerObject*)avatar->getParent();
	}

	return FALSE;
}

class LLAttachmentEnableDetach : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = enable_detach();
		return new_value;
	}
};

// Used to tell if the selected object can be attached to your avatar.
//BOOL object_selected_and_point_valid()
// [RLVa:KB] - Checked: 2010-03-16 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
BOOL object_selected_and_point_valid(const LLSD& sdParam)
// [/RLVa:KB]
{
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
	if (rlv_handler_t::isEnabled())
	{
		if (!isAgentAvatarValid())
			return FALSE;

		// RELEASE-RLVa: [SL-2.2.0] Look at the caller graph for this function on every new release
		//   - object_is_wearable() => dead code [sdParam == 0 => default attach point => OK!]
		//   - enabler set up in LLVOAvatarSelf::buildMenus() => Rezzed prim / Put On / "Attach To" [sdParam == idxAttachPt]
		//   - "Object.EnableWear" enable => Rezzed prim / Put On / "Wear" or "Add" [sdParam blank]
		// RELEASE-RLVa: [SL-2.2.0] If 'idxAttachPt != 0' then the object will be "add attached" [see LLSelectMgr::sendAttach()]
		const LLViewerJointAttachment* pAttachPt = 
			get_if_there(gAgentAvatarp->mAttachmentPoints, sdParam.asInteger(), (LLViewerJointAttachment*)NULL);
		if ( ((!pAttachPt) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_ANY))) ||		// Can't wear on default attach point
			 ((pAttachPt) && ((RLV_WEAR_ADD & gRlvAttachmentLocks.canAttach(pAttachPt)) == 0)) ||	// or non-attachable attach point
			 (gRlvHandler.hasBehaviour(RLV_BHVR_REZ)) )												// Attach on object == "Take"
		{
			return FALSE;
		}
	}
// [/RLVa:KB]

	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
	for (LLObjectSelection::root_iterator iter = selection->root_begin();
		 iter != selection->root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* object = node->getObject();
		LLViewerObject::const_child_list_t& child_list = object->getChildren();
		for (LLViewerObject::child_list_t::const_iterator iter = child_list.begin();
			 iter != child_list.end(); iter++)
		{
			LLViewerObject* child = *iter;
			if (child->isAvatar())
			{
				return FALSE;
			}
		}
	}

	return (selection->getRootObjectCount() == 1) && 
		(selection->getFirstRootObject()->getPCode() == LL_PCODE_VOLUME) && 
		selection->getFirstRootObject()->permYouOwner() &&
		selection->getFirstRootObject()->flagObjectMove() &&
		!selection->getFirstRootObject()->flagObjectPermanent() &&
		!((LLViewerObject*)selection->getFirstRootObject()->getRoot())->isAvatar() && 
		(selection->getFirstRootObject()->getNVPair("AssetContainer") == NULL);
}


// [RLVa:KB] - Checked: 2010-03-16 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
/*
BOOL object_is_wearable()
{
//	if (!object_selected_and_point_valid())
	if (!object_selected_and_point_valid(LLSD(0)))
	{
		return FALSE;
	}
	if (sitting_on_selection())
	{
		return FALSE;
	}
	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
	for (LLObjectSelection::valid_root_iterator iter = LLSelectMgr::getInstance()->getSelection()->valid_root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->valid_root_end(); iter++)
	{
		LLSelectNode* node = *iter;		
		if (node->mPermissions->getOwner() == gAgent.getID())
		{
			return TRUE;
		}
	}
	return FALSE;
}
*/
// [/RLVa:KB]

class LLAttachmentPointFilled : public view_listener_t
{
	bool handleEvent(const LLSD& user_data)
	{
		bool enable = false;
		LLVOAvatar::attachment_map_t::iterator found_it = gAgentAvatarp->mAttachmentPoints.find(user_data.asInteger());
		if (found_it != gAgentAvatarp->mAttachmentPoints.end())
		{
//			enable = found_it->second->getNumObjects() > 0;
// [RLVa:KB] - Checked: 2010-03-04 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
			// Enable the option if there is at least one attachment on this attachment point that can be detached
			enable = (found_it->second->getNumObjects() > 0) && 
				((!rlv_handler_t::isEnabled()) || (gRlvAttachmentLocks.canDetach(found_it->second)));
// [/RLVa:KB]
		}
		return enable;
	}
};

class LLAvatarSendIM : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
//		if(avatar)
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) )
// [/RLVa:KB]
		{
			LLAvatarActions::startIM(avatar->getID());
		}
		return true;
	}
};

class LLAvatarCall : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object( LLSelectMgr::getInstance()->getSelection()->getPrimaryObject() );
//		if(avatar)
// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (avatar) && (RlvActions::canShowName(RlvActions::SNC_DEFAULT, avatar->getID())) )
// [/RLVa:KB]
		{
			LLAvatarActions::startCall(avatar->getID());
		}
		return true;
	}
};

// [RLVa:KB] - Checked: RLVa-1.2.1
bool enable_avatar_call()
{
	if (RlvActions::isRlvEnabled())
	{
		const LLVOAvatar* pAvatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if ((!pAvatar) || (!RlvActions::canShowName(RlvActions::SNC_DEFAULT, pAvatar->getID())))
			return false;
	}
	return LLAvatarActions::canCall();
}
// [/RLVa:KB]

namespace
{
	struct QueueObjects : public LLSelectedNodeFunctor
	{
		BOOL scripted;
		BOOL modifiable;
		LLFloaterScriptQueue* mQueue;
		QueueObjects(LLFloaterScriptQueue* q) : mQueue(q), scripted(FALSE), modifiable(FALSE) {}
		virtual bool apply(LLSelectNode* node)
		{
			LLViewerObject* obj = node->getObject();
			if (!obj)
			{
				return true;
			}
			scripted = obj->flagScripted();
			modifiable = obj->permModify();

			if( scripted && modifiable )
			{
				mQueue->addObject(obj->getID(), node->mName);
				return false;
			}
			else
			{
				return true; // fail: stop applying
			}
		}
	};
}

void queue_actions(LLFloaterScriptQueue* q, const std::string& msg)
{
	QueueObjects func(q);
	LLSelectMgr *mgr = LLSelectMgr::getInstance();
	LLObjectSelectionHandle selectHandle = mgr->getSelection();
	bool fail = selectHandle->applyToNodes(&func);
	if(fail)
	{
		if ( !func.scripted )
		{
			std::string noscriptmsg = std::string("Cannot") + msg + "SelectObjectsNoScripts";
			LLNotificationsUtil::add(noscriptmsg);
		}
		else if ( !func.modifiable )
		{
			std::string nomodmsg = std::string("Cannot") + msg + "SelectObjectsNoPermission";
			LLNotificationsUtil::add(nomodmsg);
		}
		else
		{
			LL_ERRS() << "Bad logic." << LL_ENDL;
		}
	}
	else
	{
		if (!q->start())
		{
			LL_WARNS() << "Unexpected script compile failure." << LL_ENDL;
		}
	}
}

class LLToolsSelectedScriptAction : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-04-19 (RLVa-1.2.0f) | Modified: RLVa-1.0.5a
		// We'll allow resetting the scripts of objects on a non-attachable attach point since they wouldn't be able to circumvent anything
		if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) )
		{
			LLObjectSelectionHandle hSel = LLSelectMgr::getInstance()->getSelection();
			RlvSelectHasLockedAttach f;
			if ( (hSel->isAttachment()) && (hSel->getFirstNode(&f) != NULL) )
				return true;
		}
// [/RLVa:KB]

		std::string action = userdata.asString();
		bool mono = false;
		std::string msg, name;
		std::string title;
		if (action == "compile mono")
		{
			name = "compile_queue";
			mono = true;
			msg = "Recompile";
			title = LLTrans::getString("CompileQueueTitle");
		}
		if (action == "compile lsl")
		{
			name = "compile_queue";
			msg = "Recompile";
			title = LLTrans::getString("CompileQueueTitle");
		}
		else if (action == "reset")
		{
			name = "reset_queue";
			msg = "Reset";
			title = LLTrans::getString("ResetQueueTitle");
		}
		else if (action == "start")
		{
			name = "start_queue";
			msg = "SetRunning";
			title = LLTrans::getString("RunQueueTitle");
		}
		else if (action == "stop")
		{
			name = "stop_queue";
			msg = "SetRunningNot";
			title = LLTrans::getString("NotRunQueueTitle");
		}
		// <FS> Delete scripts
		else if (action == "delete")
		{
			name = "delete_queue";
			msg = "delete";
			title = LLTrans::getString("DeleteQueueTitle");
		}
		// </FS> Delete scripts
		LLUUID id; id.generate();
		
		LLFloaterScriptQueue* queue =LLFloaterReg::getTypedInstance<LLFloaterScriptQueue>(name, LLSD(id));
		if (queue)
		{
			queue->setMono(mono);
			queue_actions(queue, msg);
			queue->setTitle(title);
		}
		else
		{
			LL_WARNS() << "Failed to generate LLFloaterScriptQueue with action: " << action << LL_ENDL;
		}
		return true;
	}
};

void handle_selected_texture_info(void*)
{
	for (LLObjectSelection::valid_iterator iter = LLSelectMgr::getInstance()->getSelection()->valid_begin();
   		iter != LLSelectMgr::getInstance()->getSelection()->valid_end(); iter++)
	{
		LLSelectNode* node = *iter;
	   	
   		std::string msg;
   		msg.assign("Texture info for: ");
   		msg.append(node->mName);
	   
   		U8 te_count = node->getObject()->getNumTEs();
   		// map from texture ID to list of faces using it
   		typedef std::map< LLUUID, std::vector<U8> > map_t;
   		map_t faces_per_texture;
   		for (U8 i = 0; i < te_count; i++)
   		{
   			if (!node->isTESelected(i)) continue;
	   
   			LLViewerTexture* img = node->getObject()->getTEImage(i);
   			LLUUID image_id = img->getID();
   			faces_per_texture[image_id].push_back(i);
   		}
   		// Per-texture, dump which faces are using it.
   		map_t::iterator it;
   		for (it = faces_per_texture.begin(); it != faces_per_texture.end(); ++it)
   		{
   			LLUUID image_id = it->first;
   			U8 te = it->second[0];
   			LLViewerTexture* img = node->getObject()->getTEImage(te);
   			S32 height = img->getHeight();
   			S32 width = img->getWidth();
   			S32 components = img->getComponents();
   			msg.append(llformat("\n%dx%d %s on face ",
   								width,
   								height,
   								(components == 4 ? "alpha" : "opaque")));
   			for (U8 i = 0; i < it->second.size(); ++i)
   			{
   				msg.append( llformat("%d ", (S32)(it->second[i])));
   			}
   		}
		// <FS:Ansariel> Report texture info to local chat instead of toasts
   		//LLSD args;
   		//args["MESSAGE"] = msg;
   		//LLNotificationsUtil::add("SystemMessage", args);
		report_to_nearby_chat(msg);
		// </FS:Ansariel>
	}
}

void handle_selected_material_info()
{
	for (LLObjectSelection::valid_iterator iter = LLSelectMgr::getInstance()->getSelection()->valid_begin();
		iter != LLSelectMgr::getInstance()->getSelection()->valid_end(); iter++)
	{
		LLSelectNode* node = *iter;
		
		std::string msg;
		msg.assign("Material info for: \n");
		msg.append(node->mName);
		
		U8 te_count = node->getObject()->getNumTEs();
		// map from material ID to list of faces using it
		typedef std::map<LLMaterialID, std::vector<U8> > map_t;
		map_t faces_per_material;
		for (U8 i = 0; i < te_count; i++)
		{
			if (!node->isTESelected(i)) continue;
	
			const LLMaterialID& material_id = node->getObject()->getTEref(i).getMaterialID();
			faces_per_material[material_id].push_back(i);
		}
		// Per-material, dump which faces are using it.
		map_t::iterator it;
		for (it = faces_per_material.begin(); it != faces_per_material.end(); ++it)
		{
			const LLMaterialID& material_id = it->first;
			msg += llformat("%s on face ", material_id.asString().c_str());
			for (U8 i = 0; i < it->second.size(); ++i)
			{
				msg.append( llformat("%d ", (S32)(it->second[i])));
			}
			msg.append("\n");
		}

		LLSD args;
		args["MESSAGE"] = msg;
		LLNotificationsUtil::add("SystemMessage", args);
	}
}

void handle_test_male(void*)
{
// [RLVa:KB] - Checked: 2010-03-19 (RLVa-1.2.0c) | Modified: RLVa-1.2.0a
	// TODO-RLVa: [RLVa-1.2.1] Is there any reason to still block this?
	if ( (rlv_handler_t::isEnabled()) && 
		 ((gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_ANY)) || (gRlvWearableLocks.hasLockedWearableType(RLV_LOCK_ANY))) )
	{
		return;
	}
// [/RLVa:KB]

	LLAppearanceMgr::instance().wearOutfitByName("Male Shape & Outfit");
	//gGestureList.requestResetFromServer( TRUE );
}

void handle_test_female(void*)
{
// [RLVa:KB] - Checked: 2010-03-19 (RLVa-1.2.0c) | Modified: RLVa-1.2.0a
	// TODO-RLVa: [RLVa-1.2.1] Is there any reason to still block this?
	if ( (rlv_handler_t::isEnabled()) && 
		 ((gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_ANY)) || (gRlvWearableLocks.hasLockedWearableType(RLV_LOCK_ANY))) )
	{
		return;
	}
// [/RLVa:KB]

	LLAppearanceMgr::instance().wearOutfitByName("Female Shape & Outfit");
	//gGestureList.requestResetFromServer( FALSE );
}

void handle_dump_attachments(void*)
{
	if(!isAgentAvatarValid()) return;

	for (LLVOAvatar::attachment_map_t::iterator iter = gAgentAvatarp->mAttachmentPoints.begin(); 
		 iter != gAgentAvatarp->mAttachmentPoints.end(); )
	{
		LLVOAvatar::attachment_map_t::iterator curiter = iter++;
		LLViewerJointAttachment* attachment = curiter->second;
		S32 key = curiter->first;
		for (LLViewerJointAttachment::attachedobjs_vec_t::iterator attachment_iter = attachment->mAttachedObjects.begin();
			 attachment_iter != attachment->mAttachedObjects.end();
			 ++attachment_iter)
		{
			LLViewerObject *attached_object = (*attachment_iter);
			BOOL visible = (attached_object != NULL &&
							attached_object->mDrawable.notNull() && 
							!attached_object->mDrawable->isRenderType(0));
			LLVector3 pos;
			if (visible) pos = attached_object->mDrawable->getPosition();
			LL_INFOS() << "ATTACHMENT " << key << ": item_id=" << attached_object->getAttachmentItemID()
					<< (attached_object ? " present " : " absent ")
					<< (visible ? "visible " : "invisible ")
					<<  " at " << pos
					<< " and " << (visible ? attached_object->getPosition() : LLVector3::zero)
					<< LL_ENDL;
		}
	}
}


// these are used in the gl menus to set control values, generically.
class LLToggleControl : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string control_name = userdata.asString();
		BOOL checked = gSavedSettings.getBOOL( control_name );
		gSavedSettings.setBOOL( control_name, !checked );
		return true;
	}
};

class LLCheckControl : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		std::string callback_data = userdata.asString();
		bool new_value = gSavedSettings.getBOOL(callback_data);
		return new_value;
	}
};

// <FS:Ansariel> Control enhancements
class LLTogglePerAccountControl : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string control_name = userdata.asString();
		BOOL checked = gSavedPerAccountSettings.getBOOL( control_name );
		gSavedPerAccountSettings.setBOOL( control_name, !checked );
		return true;
	}
};

class LLCheckPerAccountControl : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		std::string callback_data = userdata.asString();
		bool new_value = gSavedPerAccountSettings.getBOOL(callback_data);
		return new_value;
	}
};

class FSResetControl : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		std::string callback_data = userdata.asString();
		gSavedSettings.getControl(callback_data)->resetToDefault(true);
		return true;
	}
};
class FSResetPerAccountControl : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		std::string callback_data = userdata.asString();
		gSavedPerAccountSettings.getControl(callback_data)->resetToDefault(true);
		return true;
	}
};
// </FS:Ansariel> Control enhancements

// <FS:Ansariel> Reset Mesh LOD; Forcing highest LOD on each mesh briefly should fix
//               broken meshes bursted into triangles
static void reset_mesh_lod(LLVOAvatar* avatar)
{
	for (LLVOAvatar::attachment_map_t::iterator it = avatar->mAttachmentPoints.begin(); it != avatar->mAttachmentPoints.end(); it++)
	{
		LLViewerJointAttachment::attachedobjs_vec_t& att_objects = (*it).second->mAttachedObjects;

		for (LLViewerJointAttachment::attachedobjs_vec_t::iterator at_it = att_objects.begin(); at_it != att_objects.end(); at_it++)
		{
			LLViewerObject* objectp = *at_it;
			if (objectp)
			{
				if (objectp->getPCode() == LL_PCODE_VOLUME)
				{
					LLVOVolume* vol = (LLVOVolume*)objectp;
					if (vol && vol->isMesh())
					{
						vol->forceLOD(LLModel::LOD_HIGH);
					}
				}

				LLViewerObject::const_child_list_t& children = objectp->getChildren();
				for (LLViewerObject::const_child_list_t::const_iterator cit = children.begin(); cit != children.end(); cit++)
				{
					LLViewerObject* child_objectp = *cit;
					if (!child_objectp || (child_objectp->getPCode() != LL_PCODE_VOLUME))
					{
						continue;
					}

					LLVOVolume* child_vol = (LLVOVolume*)child_objectp;
					if (child_vol && child_vol->isMesh())
					{
						child_vol->forceLOD(LLModel::LOD_HIGH);
					}
				}
			}
		}
	}
}

class FSResetMeshLOD : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		LLVOAvatar* avatar = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
		if (avatar)
		{
			reset_mesh_lod(avatar);
		}

		return true;
	}
};
// </FS:Ansariel>

// not so generic

class LLAdvancedCheckRenderShadowOption: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string control_name = userdata.asString();
		S32 current_shadow_level = gSavedSettings.getS32(control_name);
		if (current_shadow_level == 0) // is off
		{
			return false;
		}
		else // is on
		{
			return true;
		}
	}
};

class LLAdvancedClickRenderShadowOption: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string control_name = userdata.asString();
		S32 current_shadow_level = gSavedSettings.getS32(control_name);
		if (current_shadow_level == 0) // upgrade to level 2
		{
			gSavedSettings.setS32(control_name, 2);
		}
		else // downgrade to level 0
		{
			gSavedSettings.setS32(control_name, 0);
		}
		return true;
	}
};

class LLAdvancedClickRenderProfile: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gShaderProfileFrame = TRUE;
		return true;
	}
};

F32 gpu_benchmark();

class LLAdvancedClickRenderBenchmark: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gpu_benchmark();
		return true;
	}
};

//[FIX FIRE-1927 - enable DoubleClickTeleport shortcut : SJ]
class LLAdvancedToggleDoubleClickTeleport: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		BOOL checked = gSavedSettings.getBOOL("DoubleClickTeleport");
		if (checked)
		{
			gSavedSettings.setBOOL("DoubleClickTeleport", FALSE);
			report_to_nearby_chat(LLTrans::getString("DoubleClickTeleportDisabled"));
		}
		else
		{
			gSavedSettings.setBOOL("DoubleClickTeleport", TRUE);
			gSavedSettings.setBOOL("DoubleClickAutoPilot", FALSE);
			report_to_nearby_chat(LLTrans::getString("DoubleClickTeleportEnabled"));
		}
		return true;
	}
};
void menu_toggle_attached_lights(void* user_data)
{
	LLPipeline::sRenderAttachedLights = gSavedSettings.getBOOL("RenderAttachedLights");
}

void menu_toggle_attached_particles(void* user_data)
{
	LLPipeline::sRenderAttachedParticles = gSavedSettings.getBOOL("RenderAttachedParticles");
}

class LLAdvancedHandleAttachedLightParticles: public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string control_name = userdata.asString();

		// toggle the control
		gSavedSettings.setBOOL(control_name,
				       !gSavedSettings.getBOOL(control_name));

		// update internal flags
		// <FS:Ansariel> Make change to RenderAttachedLights & RenderAttachedParticles instant
		//if (control_name == "RenderAttachedLights")
		//{
		//	menu_toggle_attached_lights(NULL);
		//}
		//else if (control_name == "RenderAttachedParticles")
		//{
		//	menu_toggle_attached_particles(NULL);
		//}
		// </FS:Ansariel>
		return true;
	}
};

class LLSomethingSelected : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = !(LLSelectMgr::getInstance()->getSelection()->isEmpty());
		return new_value;
	}
};

class LLSomethingSelectedNoHUD : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();
		bool new_value = !(selection->isEmpty()) && !(selection->getSelectType() == SELECT_TYPE_HUD);
		return new_value;
	}
};

static bool is_editable_selected()
{
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.0.5a
	// RELEASE-RLVa: [SL-2.2.0] Check that this still isn't called by anything but script actions in the Build menu
	if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedAttachmentPoint(RLV_LOCK_REMOVE)) )
	{
		LLObjectSelectionHandle hSelection = LLSelectMgr::getInstance()->getSelection();

		// NOTE: this is called for 5 different menu items so we'll trade accuracy for efficiency and only
		//       examine root nodes (LLToolsSelectedScriptAction::handleEvent() will catch what we miss)
		RlvSelectHasLockedAttach f;
		if ( (hSelection->isAttachment()) && (hSelection->getFirstRootNode(&f)) )
		{
			return false;
		}
	}
// [/RLVa:KB]

	return (LLSelectMgr::getInstance()->getSelection()->getFirstEditableObject() != NULL);
}

class LLEditableSelected : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return is_editable_selected();
	}
};

class LLEditableSelectedMono : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = false;
		LLViewerRegion* region = gAgent.getRegion();
		if(region && gMenuHolder)
		{
			bool have_cap = (! region->getCapability("UpdateScriptTask").empty());
			new_value = is_editable_selected() && have_cap;
		}
		return new_value;
	}
};

bool enable_object_take_copy()
{
	bool all_valid = false;
	if (LLSelectMgr::getInstance())
	{
		if (!LLSelectMgr::getInstance()->getSelection()->isEmpty())
		{
		all_valid = true;
#ifndef HACKED_GODLIKE_VIEWER
# ifdef TOGGLE_HACKED_GODLIKE_VIEWER
		if (!LLGridManager::getInstance()->isInSLBeta()
            || !gAgent.isGodlike())
# endif
		{
			struct f : public LLSelectedObjectFunctor
			{
				virtual bool apply(LLViewerObject* obj)
				{
//					return (!obj->permCopy() || obj->isAttachment());
// [RLVa:KB] - Checked: 2010-04-01 (RLVa-1.2.0c) | Modified: RLVa-1.0.0g
					return (!obj->permCopy() || obj->isAttachment()) || 
						( (gRlvHandler.hasBehaviour(RLV_BHVR_UNSIT)) && (isAgentAvatarValid()) && (gAgentAvatarp->getRoot() == obj) );
// [/RLVa:KB]
				}
			} func;
			const bool firstonly = true;
			bool any_invalid = LLSelectMgr::getInstance()->getSelection()->applyToRootObjects(&func, firstonly);
			all_valid = !any_invalid;
		}
#endif // HACKED_GODLIKE_VIEWER
		}
	}

	return all_valid;
}


class LLHasAsset : public LLInventoryCollectFunctor
{
public:
	LLHasAsset(const LLUUID& id) : mAssetID(id), mHasAsset(FALSE) {}
	virtual ~LLHasAsset() {}
	virtual bool operator()(LLInventoryCategory* cat,
							LLInventoryItem* item);
	BOOL hasAsset() const { return mHasAsset; }

protected:
	LLUUID mAssetID;
	BOOL mHasAsset;
};

bool LLHasAsset::operator()(LLInventoryCategory* cat,
							LLInventoryItem* item)
{
	if(item && item->getAssetUUID() == mAssetID)
	{
		mHasAsset = TRUE;
	}
	return FALSE;
}


BOOL enable_save_into_task_inventory(void*)
{
	LLSelectNode* node = LLSelectMgr::getInstance()->getSelection()->getFirstRootNode();
	if(node && (node->mValid) && (!node->mFromTaskID.isNull()))
	{
		// *TODO: check to see if the fromtaskid object exists.
		LLViewerObject* obj = node->getObject();
		if( obj && !obj->isAttachment() )
		{
			return TRUE;
		}
	}
	return FALSE;
}

class LLToolsEnableSaveToObjectInventory : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = enable_save_into_task_inventory(NULL);
		return new_value;
	}
};

class LLToggleHowTo : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFloaterWebContent::Params p;
		std::string url = gSavedSettings.getString("HowToHelpURL");
		p.url = LLWeb::expandURLSubstitutions(url, LLSD());
		p.show_chrome = false;
		p.target = "__help_how_to";
		p.show_page_title = false;
		p.preferred_media_size = LLRect(0, 460, 335, 0);

		LLFloaterReg::toggleInstanceOrBringToFront("how_to", p);
		return true;
	}
};

class LLViewEnableMouselook : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// You can't go directly from customize avatar to mouselook.
		// TODO: write code with appropriate dialogs to handle this transition.
		bool new_value = (CAMERA_MODE_CUSTOMIZE_AVATAR != gAgentCamera.getCameraMode() && !gSavedSettings.getBOOL("FreezeTime"));
		return new_value;
	}
};

class LLToolsEnableToolNotPie : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = ( LLToolMgr::getInstance()->getBaseTool() != LLToolPie::getInstance() );
		return new_value;
	}
};

class LLWorldEnableCreateLandmark : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
//		return !LLLandmarkActions::landmarkAlreadyExists();
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.4.5) | Added: RLVa-1.2.1
		return (!LLLandmarkActions::landmarkAlreadyExists()) && (!gRlvHandler.hasBehaviour(RLV_BHVR_SHOWLOC));
// [/RLVa:KB]
	}
};

class LLWorldEnableSetHomeLocation : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gAgent.isGodlike() || 
			(gAgent.getRegion() && gAgent.getRegion()->getAllowSetHome());
		return new_value;
	}
};

class LLWorldEnableTeleportHome : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLViewerRegion* regionp = gAgent.getRegion();
		bool agent_on_prelude = (regionp && regionp->isPrelude());
		bool enable_teleport_home = gAgent.isGodlike() || !agent_on_prelude;
// [RLVa:KB] - Checked: 2010-09-28 (RLVa-1.2.1f) | Modified: RLVa-1.2.1f
		enable_teleport_home &= 
			(!rlv_handler_t::isEnabled()) || ((!gRlvHandler.hasBehaviour(RLV_BHVR_TPLM)) && (!gRlvHandler.hasBehaviour(RLV_BHVR_TPLOC)));
// [/RLVa:KB]
		return enable_teleport_home;
	}
};

BOOL enable_god_full(void*)
{
	return gAgent.getGodLevel() >= GOD_FULL;
}

BOOL enable_god_liaison(void*)
{
	return gAgent.getGodLevel() >= GOD_LIAISON;
}

bool is_god_customer_service()
{
	return gAgent.getGodLevel() >= GOD_CUSTOMER_SERVICE;
}

BOOL enable_god_basic(void*)
{
	return gAgent.getGodLevel() > GOD_NOT;
}


void toggle_show_xui_names(void *)
{
	gSavedSettings.setBOOL("DebugShowXUINames", !gSavedSettings.getBOOL("DebugShowXUINames"));
}

BOOL check_show_xui_names(void *)
{
	return gSavedSettings.getBOOL("DebugShowXUINames");
}

// <FS:CR> Resync Animations
class FSToolsResyncAnimations : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		for (U32 i = 0; i < gObjectList.getNumObjects(); i++)
		{
			LLViewerObject* object = gObjectList.getObject(i);
			if (object &&
				object->isAvatar())
			{
				LLVOAvatar* avatarp = (LLVOAvatar*)object;
				if (avatarp)
				{
					for (LLVOAvatar::AnimIterator anim_it = avatarp->mPlayingAnimations.begin();
						 anim_it != avatarp->mPlayingAnimations.end();
						 anim_it++)
					{
						avatarp->stopMotion(anim_it->first, TRUE);
						avatarp->startMotion(anim_it->first);
					}
				}
			}
		}
		return true;
	}
};
// </FS:CR> Resync Animations

// <FS:CR> FIRE-4345: Undeform
class FSToolsUndeform : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (isAgentAvatarValid())
		{
			gAgentAvatarp->resetSkeleton(true);

			FSPose::getInstance()->setPose(gSavedSettings.getString("FSUndeformUUID"), false);
			gAgentAvatarp->updateVisualParams();
		}

		return true;
	}
};
// </FS:CR> FIRE-4345: Undeform

// <FS:CR> Stream list import/export
class FSStreamListExportXML :public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFilePicker& file_picker = LLFilePicker::instance();
		if(file_picker.getSaveFile(LLFilePicker::FFSAVE_XML, LLDir::getScrubbedFileName("stream_list.xml")))
		{
			std::string filename = file_picker.getFirstFile();
			llofstream export_file(filename.c_str());
			LLSDSerialize::toPrettyXML(gSavedSettings.getLLSD("FSStreamList"), export_file);
			export_file.close();
			LLSD args;
			args["FILENAME"] = filename;
			LLNotificationsUtil::add("StreamListExportSuccess", args);
		}
		else
			LL_INFOS() << "User closed the filepicker. Aborting!" << LL_ENDL;

		return true;
	}
};

class FSStreamListImportXML :public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFilePicker& file_picker = LLFilePicker::instance();
		if(file_picker.getOpenFile(LLFilePicker::FFLOAD_XML))
		{
			std::string filename = file_picker.getFirstFile();
			llifstream stream_list(filename.c_str());
			if(!stream_list.is_open())
			{
				LL_WARNS() << "Couldn't open the xml file for reading. Aborting import!" << LL_ENDL;
				return true;
			}
			LLSD stream_data;
			if(LLSDSerialize::fromXML(stream_data, stream_list) >= 1)
			{
				gSavedSettings.setLLSD("FSStreamList", stream_data);
				LLNotificationsUtil::add("StreamListImportSuccess");
			}
			stream_list.close();
		}
		
		return true;
	}
};
// </FS:CR> Stream list import/export

// <FS:CR> Dump SimulatorFeatures to chat
class FSDumpSimulatorFeaturesToChat : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (LLViewerRegion* region = gAgent.getRegion())
		{
			LLSD sim_features;
			std::stringstream out_str;
			region->getSimulatorFeatures(sim_features);
			LLSDSerialize::toPrettyXML(sim_features, out_str);
			report_to_nearby_chat(out_str.str());
		}
		return true;
	}
};
// </FS:CR> Dump SimulatorFeatures to chat

// <FS:CR> Add to contact set
class FSAddToContactSet : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		if (!rlv_handler_t::isEnabled() || !gRlvHandler.hasBehaviour(RLV_BHVR_SHOWNAMES))
		{
			LLVOAvatar* avatarp = find_avatar_from_object(LLSelectMgr::getInstance()->getSelection()->getPrimaryObject());
			if (avatarp)
			{
				LLFloaterReg::showInstance("fs_add_contact", LLSD(avatarp->getID()), TRUE);
			}
		}
		return true;
	}
};
// </FS:CR> Add to contact set

// <FS:CR> Opensim menu item visibility control
bool checkIsGrid(const LLSD& userdata)
{
	std::string grid_type = userdata.asString();
	if ("secondlife" == grid_type)
	{
		return LLGridManager::getInstance()->isInSecondLife();
	}
#ifdef OPENSIM
	else if ("opensim" == grid_type)
	{
		return LLGridManager::getInstance()->isInOpenSim();
	}
	else if ("aurorasim" == grid_type)
	{
		return LLGridManager::getInstance()->isInAuroraSim();
	}
#else // !OPENSIM
	else if ("opensim" == grid_type || "aurorasim" == grid_type)
	{
		LL_DEBUGS("ViewerMenu") << grid_type << "is not a supported platform on Havok builds. Disabling item." << LL_ENDL;
		return false;
	}
#endif // OPENSIM
	else
	{
		LL_WARNS("ViewerMenu") << "Unhandled or bad on_visible gridcheck parameter! (" << grid_type << ")" << LL_ENDL;
	}
	return true;
}

bool isGridFeatureEnabled(const LLSD& userdata)
{
	if (LFSimFeatureHandler::instanceExists())
	{
		const std::string feature = userdata.asString();

		if (feature == "avatar_picker")
		{
			return LFSimFeatureHandler::instance().hasAvatarPicker();
		}
		else if (feature == "destination_guide")
		{
			return LFSimFeatureHandler::instance().hasDestinationGuide();
		}
		else
		{
			LL_WARNS("ViewerMenu") << "Unhandled or bad grid feature check parameter! (" << feature << ")" << LL_ENDL;
		}
	}

	return false;
}
// </FS:CR>

// <FS:Ansariel> FIRE-21236 - Help Menu - Check Grid Status doesn't open using External Browser
void openGridStatus()
{
	if (LLWeb::useExternalBrowser(DEFAULT_GRID_STATUS_URL))
	{
		LLWeb::loadURLExternal(DEFAULT_GRID_STATUS_URL);
	}
	else
	{
		LLFloaterReg::toggleInstance("grid_status");
	}
}
// </FS:Ansariel>

class LLToolsSelectOnlyMyObjects : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		BOOL cur_val = gSavedSettings.getBOOL("SelectOwnedOnly");

		gSavedSettings.setBOOL("SelectOwnedOnly", ! cur_val );

		return true;
	}
};

class LLToolsSelectOnlyMovableObjects : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		BOOL cur_val = gSavedSettings.getBOOL("SelectMovableOnly");

		gSavedSettings.setBOOL("SelectMovableOnly", ! cur_val );

		return true;
	}
};

class LLToolsSelectBySurrounding : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLSelectMgr::sRectSelectInclusive = !LLSelectMgr::sRectSelectInclusive;

		gSavedSettings.setBOOL("RectangleSelectInclusive", LLSelectMgr::sRectSelectInclusive);
		return true;
	}
};

class LLToolsShowHiddenSelection : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// TomY TODO Merge these
		LLSelectMgr::sRenderHiddenSelections = !LLSelectMgr::sRenderHiddenSelections;

		gSavedSettings.setBOOL("RenderHiddenSelections", LLSelectMgr::sRenderHiddenSelections);
		return true;
	}
};

class LLToolsShowSelectionLightRadius : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		// TomY TODO merge these
		LLSelectMgr::sRenderLightRadius = !LLSelectMgr::sRenderLightRadius;

		gSavedSettings.setBOOL("RenderLightRadius", LLSelectMgr::sRenderLightRadius);
		return true;
	}
};

class LLToolsEditLinkedParts : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		BOOL select_individuals = !gSavedSettings.getBOOL("EditLinkedParts");
		gSavedSettings.setBOOL( "EditLinkedParts", select_individuals );
		if (select_individuals)
		{
			LLSelectMgr::getInstance()->demoteSelectionToIndividuals();
		}
		else
		{
			LLSelectMgr::getInstance()->promoteSelectionToRoot();
		}
		return true;
	}
};

void reload_vertex_shader(void *)
{
	//THIS WOULD BE AN AWESOME PLACE TO RELOAD SHADERS... just a thought	- DaveP
}

void handle_dump_avatar_local_textures(void*)
{
	gAgentAvatarp->dumpLocalTextures();
}

void handle_dump_timers()
{
	LLTrace::BlockTimer::dumpCurTimes();
}

void handle_debug_avatar_textures(void*)
{
	LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
	if (objectp)
	{
		LLFloaterReg::showInstance( "avatar_textures", LLSD(objectp->getID()) );
	}
}

void handle_grab_baked_texture(void* data)
{
	EBakedTextureIndex baked_tex_index = (EBakedTextureIndex)((intptr_t)data);
	if (!isAgentAvatarValid()) return;

	const LLUUID& asset_id = gAgentAvatarp->grabBakedTexture(baked_tex_index);
	LL_INFOS("texture") << "Adding baked texture " << asset_id << " to inventory." << LL_ENDL;
	LLAssetType::EType asset_type = LLAssetType::AT_TEXTURE;
	LLInventoryType::EType inv_type = LLInventoryType::IT_TEXTURE;
	const LLUUID folder_id = gInventory.findCategoryUUIDForType(LLFolderType::assetTypeToFolderType(asset_type));
	if(folder_id.notNull())
	{
		std::string name;
		name = "Baked " + LLAvatarAppearanceDictionary::getInstance()->getBakedTexture(baked_tex_index)->mNameCapitalized + " Texture";

		LLUUID item_id;
		item_id.generate();
		LLPermissions perm;
		perm.init(gAgentID,
				  gAgentID,
				  LLUUID::null,
				  LLUUID::null);
		U32 next_owner_perm = PERM_MOVE | PERM_TRANSFER;
		perm.initMasks(PERM_ALL,
					   PERM_ALL,
					   PERM_NONE,
					   PERM_NONE,
					   next_owner_perm);
		time_t creation_date_now = time_corrected();
		LLPointer<LLViewerInventoryItem> item
			= new LLViewerInventoryItem(item_id,
										folder_id,
										perm,
										asset_id,
										asset_type,
										inv_type,
										name,
										LLStringUtil::null,
										LLSaleInfo::DEFAULT,
										LLInventoryItemFlags::II_FLAGS_NONE,
										creation_date_now);

		item->updateServer(TRUE);
		gInventory.updateItem(item);
		gInventory.notifyObservers();

		// Show the preview panel for textures to let
		// user know that the image is now in inventory.
		LLInventoryPanel *active_panel = LLInventoryPanel::getActiveInventoryPanel();
		if(active_panel)
		{
			LLFocusableElement* focus_ctrl = gFocusMgr.getKeyboardFocus();

			active_panel->setSelection(item_id, TAKE_FOCUS_NO);
			active_panel->openSelected();
			//LLFloaterInventory::dumpSelectionInformation((void*)view);
			// restore keyboard focus
			gFocusMgr.setKeyboardFocus(focus_ctrl);
		}
	}
	else
	{
		LL_WARNS() << "Can't find a folder to put it in" << LL_ENDL;
	}
}

BOOL enable_grab_baked_texture(void* data)
{
	EBakedTextureIndex index = (EBakedTextureIndex)((intptr_t)data);
	if (isAgentAvatarValid())
	{
		return gAgentAvatarp->canGrabBakedTexture(index);
	}
	return FALSE;
}

// Returns a pointer to the avatar give the UUID of the avatar OR of an attachment the avatar is wearing.
// Returns NULL on failure.
LLVOAvatar* find_avatar_from_object( LLViewerObject* object )
{
	if (object)
	{
		if( object->isAttachment() )
		{
			do
			{
				object = (LLViewerObject*) object->getParent();
			}
			while( object && !object->isAvatar() );
		}
		else if( !object->isAvatar() )
		{
			object = NULL;
		}
	}

	return (LLVOAvatar*) object;
}


// Returns a pointer to the avatar give the UUID of the avatar OR of an attachment the avatar is wearing.
// Returns NULL on failure.
LLVOAvatar* find_avatar_from_object( const LLUUID& object_id )
{
	return find_avatar_from_object( gObjectList.findObject(object_id) );
}


void handle_disconnect_viewer(void *)
{
	LLAppViewer::instance()->forceDisconnect(LLTrans::getString("TestingDisconnect"));
}

void force_error_breakpoint(void *)
{
    LLAppViewer::instance()->forceErrorBreakpoint();
}

void force_error_llerror(void *)
{
    LLAppViewer::instance()->forceErrorLLError();
}

void force_error_bad_memory_access(void *)
{
    LLAppViewer::instance()->forceErrorBadMemoryAccess();
}

void force_error_infinite_loop(void *)
{
    LLAppViewer::instance()->forceErrorInfiniteLoop();
}

void force_error_software_exception(void *)
{
    LLAppViewer::instance()->forceErrorSoftwareException();
}

void force_error_driver_crash(void *)
{
    LLAppViewer::instance()->forceErrorDriverCrash();
}

class LLToolsUseSelectionForGrid : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLSelectMgr::getInstance()->clearGridObjects();
		struct f : public LLSelectedObjectFunctor
		{
			virtual bool apply(LLViewerObject* objectp)
			{
				LLSelectMgr::getInstance()->addGridObject(objectp);
				return true;
			}
		} func;
		LLSelectMgr::getInstance()->getSelection()->applyToRootObjects(&func);
		LLSelectMgr::getInstance()->setGridMode(GRID_MODE_REF_OBJECT);
		LLFloaterTools::setGridMode((S32)GRID_MODE_REF_OBJECT);
		return true;
	}
};

void handle_test_load_url(void*)
{
	LLWeb::loadURL("");
	LLWeb::loadURL("hacker://www.google.com/");
	LLWeb::loadURL("http");
	LLWeb::loadURL("http://www.google.com/");
}

//
// LLViewerMenuHolderGL
//
static LLDefaultChildRegistry::Register<LLViewerMenuHolderGL> r("menu_holder");

LLViewerMenuHolderGL::LLViewerMenuHolderGL(const LLViewerMenuHolderGL::Params& p)
: LLMenuHolderGL(p)
{}

BOOL LLViewerMenuHolderGL::hideMenus()
{
	BOOL handled = FALSE;
	
	if (LLMenuHolderGL::hideMenus())
	{
		LLToolPie::instance().blockClickToWalk();
		handled = TRUE;
	}

	// drop pie menu selection
	mParcelSelection = NULL;
	mObjectSelection = NULL;

	if (gMenuBarView)
	{
		gMenuBarView->clearHoverItem();
		gMenuBarView->resetMenuTrigger();
	}

	return handled;
}

void LLViewerMenuHolderGL::setParcelSelection(LLSafeHandle<LLParcelSelection> selection) 
{ 
	mParcelSelection = selection; 
}

void LLViewerMenuHolderGL::setObjectSelection(LLSafeHandle<LLObjectSelection> selection) 
{ 
	mObjectSelection = selection; 
}


const LLRect LLViewerMenuHolderGL::getMenuRect() const
{
	return LLRect(0, getRect().getHeight() - MENU_BAR_HEIGHT, getRect().getWidth(), STATUS_BAR_HEIGHT);
}

void handle_save_to_xml(void*)
{
	LLFloater* frontmost = gFloaterView->getFrontmost();
	if (!frontmost)
	{
        LLNotificationsUtil::add("NoFrontmostFloater");
		return;
	}

	std::string default_name = "floater_";
	default_name += frontmost->getTitle();
	default_name += ".xml";

	LLStringUtil::toLower(default_name);
	LLStringUtil::replaceChar(default_name, ' ', '_');
	LLStringUtil::replaceChar(default_name, '/', '_');
	LLStringUtil::replaceChar(default_name, ':', '_');
	LLStringUtil::replaceChar(default_name, '"', '_');

	LLFilePicker& picker = LLFilePicker::instance();
	if (picker.getSaveFile(LLFilePicker::FFSAVE_XML, default_name))
	{
		std::string filename = picker.getFirstFile();
		LLUICtrlFactory::getInstance()->saveToXML(frontmost, filename);
	}
}

void handle_load_from_xml(void*)
{
	LLFilePicker& picker = LLFilePicker::instance();
	if (picker.getOpenFile(LLFilePicker::FFLOAD_XML))
	{
		std::string filename = picker.getFirstFile();
		LLFloater* floater = new LLFloater(LLSD());
		floater->buildFromFile(filename);
	}
}

void handle_web_browser_test(const LLSD& param)
{
	std::string url = param.asString();
	if (url.empty())
	{
		url = "about:blank";
	}
	LLWeb::loadURLInternal(url);
}

bool callback_clear_cache_immediately(const LLSD& notification, const LLSD& response)
{
	S32 option = LLNotificationsUtil::getSelectedOption(notification, response);
	if ( option == 0 ) // YES
	{
		//clear cache
		LLAppViewer::instance()->purgeCacheImmediate();
	}

	return false;
}

void handle_cache_clear_immediately()
{
	LLNotificationsUtil::add("ConfirmClearCache", LLSD(), LLSD(), callback_clear_cache_immediately);
}

void handle_web_content_test(const LLSD& param)
{
	std::string url = param.asString();
	// <FS:LO> Add a user settable home page for the built in web browser
	if (url == "HOME_PAGE")
	{
		url = gSavedSettings.getString("FSBrowserHomePage");
	}
	// </FS:LO>
	LLWeb::loadURLInternal(url, LLStringUtil::null, LLStringUtil::null, true);
}

void handle_show_url(const LLSD& param)
{
	std::string url = param.asString();
	if (LLWeb::useExternalBrowser(url))
	{
		LLWeb::loadURLExternal(url);
	}
	else
	{
		LLWeb::loadURLInternal(url);
	}

}

void handle_report_bug(const LLSD& param)
{
	LLUIString url(param.asString());
	
	LLStringUtil::format_map_t replace;
	// <FS:Ansariel> FIRE-14001: JIRA report is being cut off when using Help -> Report Bug
	//std::string environment = LLAppViewer::instance()->getViewerInfoString(true);
	//boost::regex regex;
	//regex.assign("</?nolink>");
	//std::string stripped_env = boost::regex_replace(environment, regex, "");

	//replace["[ENVIRONMENT]"] = LLURI::escape(stripped_env);
	LLSD sysinfo = FSData::getSystemInfo();
	replace["[ENVIRONMENT]"] = LLURI::escape(sysinfo["Part1"].asString().substr(1) + sysinfo["Part2"].asString().substr(1));
	// </FS:Ansariel>
	LLSLURL location_url;
	LLAgentUI::buildSLURL(location_url);
	replace["[LOCATION]"] = LLURI::escape(location_url.getSLURLString());

	LLUIString file_bug_url = gSavedSettings.getString("ReportBugURL");
	file_bug_url.setArgs(replace);

	LLWeb::loadURLExternal(file_bug_url.getString());
}

void handle_buy_currency_test(void*)
{
	std::string url =
		"http://sarahd-sl-13041.webdev.lindenlab.com/app/lindex/index.php?agent_id=[AGENT_ID]&secure_session_id=[SESSION_ID]&lang=[LANGUAGE]";

	LLStringUtil::format_map_t replace;
	replace["[AGENT_ID]"] = gAgent.getID().asString();
	replace["[SESSION_ID]"] = gAgent.getSecureSessionID().asString();
	replace["[LANGUAGE]"] = LLUI::getLanguage();
	LLStringUtil::format(url, replace);

	LL_INFOS() << "buy currency url " << url << LL_ENDL;

	LLFloaterReg::showInstance("buy_currency_html", LLSD(url));
}

//-- SUNSHINE CLEANUP - is only the request update at the end needed now?
void handle_rebake_textures(void*)
{
	if (!isAgentAvatarValid()) return;

	// Slam pending upload count to "unstick" things
	bool slam_for_debug = true;
	gAgentAvatarp->forceBakeAllTextures(slam_for_debug);
	if (gAgent.getRegion() && gAgent.getRegion()->getCentralBakeVersion())
	{
// [SL:KB] - Patch: Appearance-Misc | Checked: 2015-06-27 (Catznip-3.7)
		if (!gAgent.getRegionCapability("IncrementCOFVersion").empty())
		{
			LLAppearanceMgr::instance().syncCofVersionAndRefresh();
		}
		else
		{
			LLAppearanceMgr::instance().requestServerAppearanceUpdate();
		}
// [/SL:KB]
//		LLAppearanceMgr::instance().requestServerAppearanceUpdate();
		avatar_tex_refresh(gAgentAvatarp); // <FS:CR> FIRE-11800 - Refresh the textures too
	}
	reset_mesh_lod(gAgentAvatarp); // <FS:Ansariel> Reset Mesh LOD
	gAgentAvatarp->setIsCrossingRegion(false); // <FS:Ansariel> FIRE-12004: Attachments getting lost on TP
}

void toggle_visibility(void* user_data)
{
	LLView* viewp = (LLView*)user_data;
	viewp->setVisible(!viewp->getVisible());
}

BOOL get_visibility(void* user_data)
{
	LLView* viewp = (LLView*)user_data;
	return viewp->getVisible();
}

// TomY TODO: Get rid of these?
class LLViewShowHoverTips : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gSavedSettings.setBOOL("ShowHoverTips", !gSavedSettings.getBOOL("ShowHoverTips"));
		return true;
	}
};

class LLViewCheckShowHoverTips : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = gSavedSettings.getBOOL("ShowHoverTips");
		return new_value;
	}
};

// TomY TODO: Get rid of these?
class LLViewHighlightTransparent : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
//		LLDrawPoolAlpha::sShowDebugAlpha = !LLDrawPoolAlpha::sShowDebugAlpha;
// [RLVa:KB] - Checked: 2010-11-29 (RLVa-1.3.0c) | Modified: RLVa-1.3.0c
		LLDrawPoolAlpha::sShowDebugAlpha = (!LLDrawPoolAlpha::sShowDebugAlpha) && (!gRlvHandler.hasBehaviour(RLV_BHVR_EDIT));
// [/RLVa:KB]
		return true;
	}
};

class LLViewCheckHighlightTransparent : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLDrawPoolAlpha::sShowDebugAlpha;
		return new_value;
	}
};

class LLViewBeaconWidth : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string width = userdata.asString();
		if(width == "1")
		{
			gSavedSettings.setS32("DebugBeaconLineWidth", 1);
		}
		else if(width == "4")
		{
			gSavedSettings.setS32("DebugBeaconLineWidth", 4);
		}
		else if(width == "16")
		{
			gSavedSettings.setS32("DebugBeaconLineWidth", 16);
		}
		else if(width == "32")
		{
			gSavedSettings.setS32("DebugBeaconLineWidth", 32);
		}

		return true;
	}
};


class LLViewToggleBeacon : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string beacon = userdata.asString();
		if (beacon == "scriptsbeacon")
		{
			LLPipeline::toggleRenderScriptedBeacons();
			gSavedSettings.setBOOL( "scriptsbeacon", LLPipeline::getRenderScriptedBeacons() );
			// toggle the other one off if it's on
			if (LLPipeline::getRenderScriptedBeacons() && LLPipeline::getRenderScriptedTouchBeacons())
			{
				LLPipeline::toggleRenderScriptedTouchBeacons();
				gSavedSettings.setBOOL( "scripttouchbeacon", LLPipeline::getRenderScriptedTouchBeacons() );
			}
		}
		else if (beacon == "physicalbeacon")
		{
			LLPipeline::toggleRenderPhysicalBeacons();
			gSavedSettings.setBOOL( "physicalbeacon", LLPipeline::getRenderPhysicalBeacons() );
		}
		else if (beacon == "moapbeacon")
		{
			LLPipeline::toggleRenderMOAPBeacons();
			gSavedSettings.setBOOL( "moapbeacon", LLPipeline::getRenderMOAPBeacons() );
		}
		else if (beacon == "soundsbeacon")
		{
			LLPipeline::toggleRenderSoundBeacons();
			gSavedSettings.setBOOL( "soundsbeacon", LLPipeline::getRenderSoundBeacons() );
		}
		else if (beacon == "particlesbeacon")
		{
			LLPipeline::toggleRenderParticleBeacons();
			gSavedSettings.setBOOL( "particlesbeacon", LLPipeline::getRenderParticleBeacons() );
		}
		else if (beacon == "scripttouchbeacon")
		{
			LLPipeline::toggleRenderScriptedTouchBeacons();
			gSavedSettings.setBOOL( "scripttouchbeacon", LLPipeline::getRenderScriptedTouchBeacons() );
			// toggle the other one off if it's on
			if (LLPipeline::getRenderScriptedBeacons() && LLPipeline::getRenderScriptedTouchBeacons())
			{
				LLPipeline::toggleRenderScriptedBeacons();
				gSavedSettings.setBOOL( "scriptsbeacon", LLPipeline::getRenderScriptedBeacons() );
			}
		}
		else if (beacon == "renderbeacons")
		{
			LLPipeline::toggleRenderBeacons();
			gSavedSettings.setBOOL( "renderbeacons", LLPipeline::getRenderBeacons() );
			// toggle the other one on if it's not
			if (!LLPipeline::getRenderBeacons() && !LLPipeline::getRenderHighlights())
			{
				LLPipeline::toggleRenderHighlights();
				gSavedSettings.setBOOL( "renderhighlights", LLPipeline::getRenderHighlights() );
			}
		}
		else if (beacon == "renderhighlights")
		{
			LLPipeline::toggleRenderHighlights();
			gSavedSettings.setBOOL( "renderhighlights", LLPipeline::getRenderHighlights() );
			// toggle the other one on if it's not
			if (!LLPipeline::getRenderBeacons() && !LLPipeline::getRenderHighlights())
			{
				LLPipeline::toggleRenderBeacons();
				gSavedSettings.setBOOL( "renderbeacons", LLPipeline::getRenderBeacons() );
			}
		}

		return true;
	}
};

class LLViewCheckBeaconEnabled : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string beacon = userdata.asString();
		bool new_value = false;
		if (beacon == "scriptsbeacon")
		{
			new_value = gSavedSettings.getBOOL( "scriptsbeacon");
			LLPipeline::setRenderScriptedBeacons(new_value);
		}
		else if (beacon == "moapbeacon")
		{
			new_value = gSavedSettings.getBOOL( "moapbeacon");
			LLPipeline::setRenderMOAPBeacons(new_value);
		}
		else if (beacon == "physicalbeacon")
		{
			new_value = gSavedSettings.getBOOL( "physicalbeacon");
			LLPipeline::setRenderPhysicalBeacons(new_value);
		}
		else if (beacon == "soundsbeacon")
		{
			new_value = gSavedSettings.getBOOL( "soundsbeacon");
			LLPipeline::setRenderSoundBeacons(new_value);
		}
		else if (beacon == "particlesbeacon")
		{
			new_value = gSavedSettings.getBOOL( "particlesbeacon");
			LLPipeline::setRenderParticleBeacons(new_value);
		}
		else if (beacon == "scripttouchbeacon")
		{
			new_value = gSavedSettings.getBOOL( "scripttouchbeacon");
			LLPipeline::setRenderScriptedTouchBeacons(new_value);
		}
		else if (beacon == "renderbeacons")
		{
			new_value = gSavedSettings.getBOOL( "renderbeacons");
			LLPipeline::setRenderBeacons(new_value);
		}
		else if (beacon == "renderhighlights")
		{
			new_value = gSavedSettings.getBOOL( "renderhighlights");
			LLPipeline::setRenderHighlights(new_value);
		}
		return new_value;
	}
};

class LLViewToggleRenderType : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string type = userdata.asString();
		if (type == "hideparticles")
		{
			LLPipeline::toggleRenderType(LLPipeline::RENDER_TYPE_PARTICLES);
			gPipeline.sRenderParticles = gPipeline.hasRenderType(LLPipeline::RENDER_TYPE_PARTICLES);
		}
		return true;
	}
};

class LLViewCheckRenderType : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string type = userdata.asString();
		bool new_value = false;
		if (type == "hideparticles")
		{
			new_value = LLPipeline::toggleRenderTypeControlNegated(LLPipeline::RENDER_TYPE_PARTICLES);
		}
		return new_value;
	}
};

class LLViewStatusAway : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return (gAgent.isInitialized() && gAgent.getAFK());
	}
};

class LLViewStatusDoNotDisturb : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		return (gAgent.isInitialized() && gAgent.isDoNotDisturb());
	}
};

class LLViewShowHUDAttachments : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-04-19 (RLVa-1.2.1a) | Modified: RLVa-1.0.0c
		if ( (rlv_handler_t::isEnabled()) && (gRlvAttachmentLocks.hasLockedHUD()) && (LLPipeline::sShowHUDAttachments) )
			return true;
// [/RLVa:KB]

		LLPipeline::sShowHUDAttachments = !LLPipeline::sShowHUDAttachments;
		return true;
	}
};

class LLViewCheckHUDAttachments : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool new_value = LLPipeline::sShowHUDAttachments;
		return new_value;
	}
};

// <FS:Ansariel> Disable Show HUD attachments if prevented by RLVa
bool enable_show_HUD_attachments()
{
	return (!LLPipeline::sShowHUDAttachments || !rlv_handler_t::isEnabled() || !gRlvAttachmentLocks.hasLockedHUD());
};
// </FS:Ansariel>

class LLEditEnableTakeOff : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string clothing = userdata.asString();
		LLWearableType::EType type = LLWearableType::typeNameToType(clothing);
//		if (type >= LLWearableType::WT_SHAPE && type < LLWearableType::WT_COUNT)
// [RLVa:KB] - Checked: 2010-03-20 (RLVa-1.2.0c) | Modified: RLVa-1.2.0a
		// NOTE: see below - enable if there is at least one wearable on this type that can be removed
		if ( (type >= LLWearableType::WT_SHAPE && type < LLWearableType::WT_COUNT) && 
			 ((!rlv_handler_t::isEnabled()) || (gRlvWearableLocks.canRemove(type))) )
// [/RLVa:KB]
		{
			return LLAgentWearables::selfHasWearable(type);
		}
		return false;
	}
};

// <FS:Beq> Xmas present for Ansa, Animesh kill switch
class FSDerenderAnimatedObjects : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		gObjectList.killAnimatedObjects();
		return true;
	}
};

// </FS:Beq>
class LLEditTakeOff : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string clothing = userdata.asString();
		if (clothing == "all")
			LLAppearanceMgr::instance().removeAllClothesFromAvatar();
		else
		{
			LLWearableType::EType type = LLWearableType::typeNameToType(clothing);
			if (type >= LLWearableType::WT_SHAPE 
				&& type < LLWearableType::WT_COUNT
				&& (gAgentWearables.getWearableCount(type) > 0))
			{
				// MULTI-WEARABLES: assuming user wanted to remove top shirt.
				//<FS:TS> Shut the compiler up about unsigned comparisons <0 or >0
				//U32 wearable_index = gAgentWearables.getWearableCount(type) - 1;
				S32 wearable_index = gAgentWearables.getWearableCount(type) - 1;

// [RLVa:KB] - Checked: 2010-06-09 (RLVa-1.2.0g) | Added: RLVa-1.2.0g
				if ( (rlv_handler_t::isEnabled()) && (gRlvWearableLocks.hasLockedWearable(type)) )
				{
					// We'll use the first wearable we come across that can be removed (moving from top to bottom)
					for (; wearable_index >= 0; wearable_index--)
					{
						const LLViewerWearable* pWearable = gAgentWearables.getViewerWearable(type, wearable_index);
						if (!gRlvWearableLocks.isLockedWearable(pWearable))
							break;
					}
					if (wearable_index < 0)
						return true;	// No wearable found that can be removed
				}
// [/RLVa:KB]

				LLUUID item_id = gAgentWearables.getWearableItemID(type,wearable_index);
				LLAppearanceMgr::instance().removeItemFromAvatar(item_id);
			}
				
		}
		return true;
	}
};

class LLToolsSelectTool : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string tool_name = userdata.asString();
		if (tool_name == "focus")
		{
			LLToolMgr::getInstance()->getCurrentToolset()->selectToolByIndex(1);
		}
		else if (tool_name == "move")
		{
			LLToolMgr::getInstance()->getCurrentToolset()->selectToolByIndex(2);
		}
		else if (tool_name == "edit")
		{
			LLToolMgr::getInstance()->getCurrentToolset()->selectToolByIndex(3);
		}
		else if (tool_name == "create")
		{
			LLToolMgr::getInstance()->getCurrentToolset()->selectToolByIndex(4);
		}
		else if (tool_name == "land")
		{
			LLToolMgr::getInstance()->getCurrentToolset()->selectToolByIndex(5);
		}

		// Note: if floater is not visible LLViewerWindow::updateLayout() will
		// attempt to open it, but it won't bring it to front or de-minimize.
		if (gFloaterTools && (gFloaterTools->isMinimized() || !gFloaterTools->isShown() || !gFloaterTools->isFrontmost()))
		{
			gFloaterTools->setMinimized(FALSE);
			gFloaterTools->openFloater();
			gFloaterTools->setVisibleAndFrontmost(TRUE);
		}
		return true;
	}
};

/// WINDLIGHT callbacks
class LLWorldEnvSettings : public view_listener_t
{	
	bool handleEvent(const LLSD& userdata)
	{
// [RLVa:KB] - Checked: 2010-03-18 (RLVa-1.2.0a) | Modified: RLVa-1.0.0g
		if (gRlvHandler.hasBehaviour(RLV_BHVR_SETENV))
			return true;
// [/RLVa:KB]

		std::string tod = userdata.asString();
		
		if (tod == "editor")
		{
			LLFloaterReg::toggleInstance("env_settings");
			return true;
		}

		if (tod == "sunrise")
		{
			LLEnvManagerNew::instance().setUseSkyPreset("Sunrise");
		}
		else if (tod == "noon")
		{
			LLEnvManagerNew::instance().setUseSkyPreset("Midday");
		}
		else if (tod == "sunset")
		{
			LLEnvManagerNew::instance().setUseSkyPreset("Sunset");
		}
		else if (tod == "midnight")
		{
			LLEnvManagerNew::instance().setUseSkyPreset("Midnight");
		}
		else
		{
			LLEnvManagerNew &envmgr = LLEnvManagerNew::instance();
			// reset all environmental settings to track the region defaults, make this reset 'sticky' like the other sun settings.
			bool use_fixed_sky = false;
			bool use_region_settings = true;
			envmgr.setUserPrefs(envmgr.getWaterPresetName(),
					    envmgr.getSkyPresetName(),
					    envmgr.getDayCycleName(),
					    use_fixed_sky, use_region_settings, false);
		}

		return true;
	}
};

class LLWorldEnableEnvSettings : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool result = false;
		std::string tod = userdata.asString();

		if (LLEnvManagerNew::instance().getUseRegionSettings())
		{
			return (tod == "region");
		}

		if (LLEnvManagerNew::instance().getUseFixedSky())
		{
			if (tod == "sunrise")
			{
				result = (LLEnvManagerNew::instance().getSkyPresetName() == "Sunrise");
			}
			else if (tod == "noon")
			{
				result = (LLEnvManagerNew::instance().getSkyPresetName() == "Midday");
			}
			else if (tod == "sunset")
			{
				result = (LLEnvManagerNew::instance().getSkyPresetName() == "Sunset");
			}
			else if (tod == "midnight")
			{
				result = (LLEnvManagerNew::instance().getSkyPresetName() == "Midnight");
			}
			else if (tod == "region")
			{
				return false;
			}
			else
			{
				LL_WARNS() << "Unknown time-of-day item:  " << tod << LL_ENDL;
			}
		}
		return result;
	}
};

class LLWorldEnvPreset : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string item = userdata.asString();

		if (item == "new_water")
		{
			LLFloaterReg::showInstance("env_edit_water", "new");
		}
		else if (item == "edit_water")
		{
			LLFloaterReg::showInstance("env_edit_water", "edit");
		}
		else if (item == "delete_water")
		{
			LLFloaterReg::showInstance("env_delete_preset", "water");
		}
		else if (item == "new_sky")
		{
			LLFloaterReg::showInstance("env_edit_sky", "new");
		}
		else if (item == "edit_sky")
		{
			LLFloaterReg::showInstance("env_edit_sky", "edit");
		}
		else if (item == "delete_sky")
		{
			LLFloaterReg::showInstance("env_delete_preset", "sky");
		}
		else if (item == "new_day_cycle")
		{
			LLFloaterReg::showInstance("env_edit_day_cycle", "new");
		}
		else if (item == "edit_day_cycle")
		{
			LLFloaterReg::showInstance("env_edit_day_cycle", "edit");
		}
		else if (item == "delete_day_cycle")
		{
			LLFloaterReg::showInstance("env_delete_preset", "day_cycle");
		}
		else
		{
			LL_WARNS() << "Unknown item selected" << LL_ENDL;
		}

		return true;
	}
};

class LLWorldEnableEnvPreset : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		std::string item = userdata.asString();

		if (item == "delete_water")
		{
			LLWaterParamManager::preset_name_list_t user_waters;
			LLWaterParamManager::instance().getUserPresetNames(user_waters);
			return !user_waters.empty();
		}
		else if (item == "delete_sky")
		{
			LLWLParamManager::preset_name_list_t user_skies;
			LLWLParamManager::instance().getUserPresetNames(user_skies);
			return !user_skies.empty();
		}
		else if (item == "delete_day_cycle")
		{
			LLDayCycleManager::preset_name_list_t user_days;
			LLDayCycleManager::instance().getUserPresetNames(user_days);
			return !user_days.empty();
		}
		else
		{
			LL_WARNS() << "Unknown item" << LL_ENDL;
		}

		return false;
	}
};


/// Post-Process callbacks
class LLWorldPostProcess : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		LLFloaterReg::showInstance("env_post_process");
		return true;
	}
};

void handle_flush_name_caches()
{
	// <FS:Ansariel> Crash fix
	//SUBSYSTEM_CLEANUP(LLAvatarNameCache);
	LLAvatarNameCache::clearCache();
	// </FS:Ansariel>
	if (gCacheName) gCacheName->clear();
}

class LLUploadCostCalculator : public view_listener_t
{
	std::string mCostStr;

	bool handleEvent(const LLSD& userdata)
	{
		std::string menu_name = userdata.asString();
		// AW:this fights the update in llviewermessage
		calculateCost();// <FS:AW opensim currency support>
		gMenuHolder->childSetLabelArg(menu_name, "[COST]", mCostStr);

		return true;
	}

	void calculateCost();

public:
	LLUploadCostCalculator()
	{
// <FS:AW opensim currency support> we don't know the costs yet
//		calculateCost();
// </FS:AW opensim currency support>
	}
};

void handle_voice_morphing_subscribe()
{
	LLWeb::loadURL(LLTrans::getString("voice_morphing_url"));
}

void handle_premium_voice_morphing_subscribe()
{
	LLWeb::loadURL(LLTrans::getString("premium_voice_morphing_url"));
}

class LLToggleUIHints : public view_listener_t
{
	bool handleEvent(const LLSD& userdata)
	{
		bool ui_hints_enabled = gSavedSettings.getBOOL("EnableUIHints");
		// toggle
		ui_hints_enabled = !ui_hints_enabled;
		gSavedSettings.setBOOL("EnableUIHints", ui_hints_enabled);
		return true;
	}
};

void LLUploadCostCalculator::calculateCost()
{
	S32 upload_cost = LLGlobalEconomy::getInstance()->getPriceUpload();
 
 	// getPriceUpload() returns -1 if no data available yet.
// <FS:AW opensim currency support>
// 	if(upload_cost >= 0)
// 	{
// 		mCostStr = llformat("%d", upload_cost);
// 	}
// 	else
// 	{
// 		mCostStr = llformat("%d", gSavedSettings.getU32("DefaultUploadCost"));
// 	}
#ifdef OPENSIM // <FS:AW optional opensim support>
	if (LLGridManager::getInstance()->isInOpenSim())
	{
		mCostStr = upload_cost > 0 ? llformat("%s%d", "L$", upload_cost) : LLTrans::getString("free");
	}
	else
#endif // OPENSIM // <FS:AW optional opensim support>
	{
		mCostStr = "L$" + (upload_cost > 0 ? llformat("%d", upload_cost) : llformat("%d", gSavedSettings.getU32("DefaultUploadCost")));
	}
// </FS:AW opensim currency support>
}

void show_navbar_context_menu(LLView* ctrl, S32 x, S32 y)
{
	static LLMenuGL*	show_navbar_context_menu = LLUICtrlFactory::getInstance()->createFromFile<LLMenuGL>("menu_hide_navbar.xml",
			gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());
	if(gMenuHolder->hasVisibleMenu())
	{
		gMenuHolder->hideMenus();
	}
	show_navbar_context_menu->buildDrawLabels();
	show_navbar_context_menu->updateParent(LLMenuGL::sMenuContainer);
	LLMenuGL::showPopup(ctrl, show_navbar_context_menu, x, y);
}

void show_topinfobar_context_menu(LLView* ctrl, S32 x, S32 y)
{
	static LLMenuGL* show_topbarinfo_context_menu = LLUICtrlFactory::getInstance()->createFromFile<LLMenuGL>("menu_topinfobar.xml",
			gMenuHolder, LLViewerMenuHolderGL::child_registry_t::instance());

	LLMenuItemGL* landmark_item = show_topbarinfo_context_menu->getChild<LLMenuItemGL>("Landmark");
	if (!LLLandmarkActions::landmarkAlreadyExists())
	{
		landmark_item->setLabel(LLTrans::getString("AddLandmarkNavBarMenu"));
	}
	else
	{
		landmark_item->setLabel(LLTrans::getString("EditLandmarkNavBarMenu"));
	}
// [RLVa:KB] - Checked: 2012-02-07 (RLVa-1.4.5) | Added: RLVa-1.4.5
	landmark_item->setEnabled(!gRlvHandler.hasBehaviour(RLV_BHVR_SHOWLOC));
// [/RLVa:KB]

	if(gMenuHolder->hasVisibleMenu())
	{
		gMenuHolder->hideMenus();
	}

	show_topbarinfo_context_menu->buildDrawLabels();
	show_topbarinfo_context_menu->updateParent(LLMenuGL::sMenuContainer);
	LLMenuGL::showPopup(ctrl, show_topbarinfo_context_menu, x, y);
}

// <FS:Ansariel> For web browser toolbar button
void toggleWebBrowser(const LLSD& sdParam)
{
	if (LLFloaterReg::instanceVisible("web_content"))
	{
		LLFloaterReg::hideInstance("web_content");
	}
	else
	{
		std::string param = sdParam.asString();
		if (param == "HOME_PAGE")
		{
			param = gSavedSettings.getString("FSBrowserHomePage");
		}
		LLWeb::loadURLInternal(param);
	}
}
// </FS:Ansariel> For web browser toolbar button

// <FS:Ansariel> Toggle debug settings floater
void toggleSettingsDebug()
{
	LLFloaterReg::toggleInstance("settings_debug", "all");
}
// </FS:Ansariel> Toggle debug settings floater

// <FS:Ansariel> Toggle teleport history panel directly
void toggleTeleportHistory()
{
	if (gSavedSettings.getBOOL("FSUseStandaloneTeleportHistoryFloater"))
	{
		LLFloaterReg::toggleInstance("fs_teleporthistory");
	}
	else
	{
		LLFloater* floater = LLFloaterReg::findInstance("places");
		if (floater && floater->isMinimized())
		{
			floater->setMinimized(FALSE);
		}
		else if (LLFloater::isShown(floater))
		{
			LLFloaterReg::hideInstance("places");
		}
		else
		{
			LLFloaterSidePanelContainer::showPanel("places", LLSD().with("type", "open_teleport_history_tab"));
		}
	}
}
// </FS:Ansariel> Toggle teleport history panel directly

// <FS:Techwolf Lupindo> export
bool enable_export_object()
{
	for (LLObjectSelection::root_iterator iter = LLSelectMgr::getInstance()->getSelection()->root_begin();
		 iter != LLSelectMgr::getInstance()->getSelection()->root_end(); iter++)
	{
		LLSelectNode* node = *iter;
		LLViewerObject* obj = node->getObject();
		if (obj || node)
		{
			return gSavedSettings.getBOOL("FSEnableObjectExports");
		}
	}
	return false;
}

class FSObjectExport : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (objectp)
		{
			LLFloaterReg::showInstance("fs_export");
		}
		return true;
	}
};
// </FS:Techwolf Lupindo>

// <FS:CR>
class FSObjectExportCollada : public view_listener_t
{
	bool handleEvent( const LLSD& userdata)
	{
		LLViewerObject* objectp = LLSelectMgr::getInstance()->getSelection()->getPrimaryObject();
		if (objectp)
		{
			LLFloaterReg::showInstance("export_collada");
		}
		return true;
	}
};
// </FS:CR>

// <FS:Zi> Make sure to call this before any of the UI is set up, so all text editors can
//         pick up the menu properly.
void initialize_edit_menu()
{
	view_listener_t::addMenu(new LLEditUndo(), "Edit.Undo");
	view_listener_t::addMenu(new LLEditRedo(), "Edit.Redo");
	view_listener_t::addMenu(new LLEditCut(), "Edit.Cut");
	view_listener_t::addMenu(new LLEditCopy(), "Edit.Copy");
	view_listener_t::addMenu(new LLEditPaste(), "Edit.Paste");
	view_listener_t::addMenu(new LLEditDelete(), "Edit.Delete");
	view_listener_t::addMenu(new LLEditSelectAll(), "Edit.SelectAll");
	view_listener_t::addMenu(new LLEditDeselect(), "Edit.Deselect");
	view_listener_t::addMenu(new LLEditTakeOff(), "Edit.TakeOff");
	view_listener_t::addMenu(new LLEditEnableUndo(), "Edit.EnableUndo");
	view_listener_t::addMenu(new LLEditEnableRedo(), "Edit.EnableRedo");
	view_listener_t::addMenu(new LLEditEnableCut(), "Edit.EnableCut");
	view_listener_t::addMenu(new LLEditEnableCopy(), "Edit.EnableCopy");
	view_listener_t::addMenu(new LLEditEnablePaste(), "Edit.EnablePaste");
	view_listener_t::addMenu(new LLEditEnableDelete(), "Edit.EnableDelete");
	view_listener_t::addMenu(new LLEditEnableSelectAll(), "Edit.EnableSelectAll");
	view_listener_t::addMenu(new LLEditEnableDeselect(), "Edit.EnableDeselect");

}

void initialize_spellcheck_menu()
{
	LLUICtrl::CommitCallbackRegistry::Registrar& commit = LLUICtrl::CommitCallbackRegistry::currentRegistrar();
	LLUICtrl::EnableCallbackRegistry::Registrar& enable = LLUICtrl::EnableCallbackRegistry::currentRegistrar();

	commit.add("SpellCheck.ReplaceWithSuggestion", boost::bind(&handle_spellcheck_replace_with_suggestion, _1, _2));
	enable.add("SpellCheck.VisibleSuggestion", boost::bind(&visible_spellcheck_suggestion, _1, _2));
	commit.add("SpellCheck.AddToDictionary", boost::bind(&handle_spellcheck_add_to_dictionary, _1));
	enable.add("SpellCheck.EnableAddToDictionary", boost::bind(&enable_spellcheck_add_to_dictionary, _1));
	commit.add("SpellCheck.AddToIgnore", boost::bind(&handle_spellcheck_add_to_ignore, _1));
	enable.add("SpellCheck.EnableAddToIgnore", boost::bind(&enable_spellcheck_add_to_ignore, _1));
}

//<FS:KC> Centralize a some of these volume panel callbacks
static void volume_controls_open_volume_prefs()
{
	// bring up the prefs floater
	LLFloaterPreference* prefsfloater = LLFloaterReg::showTypedInstance<LLFloaterPreference>("preferences");
	if (prefsfloater)
	{
		// grab the 'audio' panel from the preferences floater and bring it the front!
		prefsfloater->selectPanel("audio");
	}
}

void volume_controls_on_click_set_sounds(const LLUICtrl* ctrl)
{
	const LLPanel* volume_control_panel = dynamic_cast<const LLPanel*>(ctrl->getParent());
	if (volume_control_panel)
	{
		// Disable Enable gesture/collisions sounds checkbox if the master sound is disabled
		// or if sound effects are disabled.

		// <FS:PP> FIRE-9856: Mute sound effects disable plays sound from collisions and plays sound from gestures checkbox not disable after restart/relog
		// volume_control_panel->getChild<LLCheckBoxCtrl>("gesture_audio_play_btn")->setEnabled(!gSavedSettings.getBOOL("MuteSounds"));
		// volume_control_panel->getChild<LLCheckBoxCtrl>("collisions_audio_play_btn")->setEnabled(!gSavedSettings.getBOOL("MuteSounds"));
		bool mute_sound_effects = gSavedSettings.getBOOL("MuteSounds");
		bool mute_all_sounds = gSavedSettings.getBOOL("MuteAudio");
		volume_control_panel->getChild<LLCheckBoxCtrl>("gesture_audio_play_btn")->setEnabled(!(mute_sound_effects || mute_all_sounds));
		volume_control_panel->getChild<LLCheckBoxCtrl>("collisions_audio_play_btn")->setEnabled(!(mute_sound_effects || mute_all_sounds));
		// </FS:PP> 

	}
}

void volume_controls_set_control_false(const LLUICtrl* ctrl, const LLSD& user_data)
{
	LLPanel* volume_control_panel = dynamic_cast<LLPanel*>(ctrl->getParent());
	if (volume_control_panel)
	{
		std::string control_name = user_data.asString();
		LLControlVariable* control = volume_control_panel->findControl(control_name);
		
		if (control)
			control->set(LLSD(FALSE));
	}
}

void initialize_volume_controls_callbacks()
{
	LLUICtrl::CommitCallbackRegistry::Registrar& commit = LLUICtrl::CommitCallbackRegistry::currentRegistrar();
	commit.add("MediaListCtrl.GoMediaPrefs",	boost::bind(&volume_controls_open_volume_prefs));
	commit.add("Pref.SetSounds",				boost::bind(&volume_controls_on_click_set_sounds, _1));
	commit.add("Pref.setControlFalse",			boost::bind(&volume_controls_set_control_false, _1, _2));
}
//</FS:KC>

// <FS:Ansariel> Force HTTP features on SL
bool use_http_inventory()
{
#ifdef OPENSIM
	return (LLGridManager::getInstance()->isInSecondLife() || gSavedSettings.getBOOL("UseHTTPInventory"));
#else
	return true;
#endif
}

bool use_http_textures()
{
#ifdef OPENSIM
	static LLCachedControl<bool> use_http(gSavedSettings, "ImagePipelineUseHTTP", true);
	return (LLGridManager::getInstance()->isInSecondLife() || use_http);
#else
	return true;
#endif
}
// <FS:Ansariel>

void initialize_menus()
{
	// A parameterized event handler used as ctrl-8/9/0 zoom controls below.
	class LLZoomer : public view_listener_t
	{
	public:
		// The "mult" parameter says whether "val" is a multiplier or used to set the value.
		LLZoomer(F32 val, bool mult=true) : mVal(val), mMult(mult) {}
		bool handleEvent(const LLSD& userdata)
		{
			F32 new_fov_rad = mMult ? LLViewerCamera::getInstance()->getDefaultFOV() * mVal : mVal;
			LLViewerCamera::getInstance()->setDefaultFOV(new_fov_rad);
			gSavedSettings.setF32("CameraAngle", LLViewerCamera::getInstance()->getView()); // setView may have clamped it.
			return true;
		}
	private:
		F32 mVal;
		bool mMult;
	};
	
	LLUICtrl::EnableCallbackRegistry::Registrar& enable = LLUICtrl::EnableCallbackRegistry::currentRegistrar();
	LLUICtrl::CommitCallbackRegistry::Registrar& commit = LLUICtrl::CommitCallbackRegistry::currentRegistrar();
	
	// Generic enable and visible
	// Don't prepend MenuName.Foo because these can be used in any menu.
	enable.add("IsGodCustomerService", boost::bind(&is_god_customer_service));

	enable.add("displayViewerEventRecorderMenuItems",boost::bind(&LLViewerEventRecorder::displayViewerEventRecorderMenuItems,&LLViewerEventRecorder::instance()));

	view_listener_t::addEnable(new LLUploadCostCalculator(), "Upload.CalculateCosts");

	// <FS:Ansariel> [FS communication UI]
	//enable.add("Conversation.IsConversationLoggingAllowed", boost::bind(&LLFloaterIMContainer::isConversationLoggingAllowed));
	
	enable.add("GridCheck", boost::bind(&checkIsGrid, _2)); // <FS:CR> Opensim menu item visibility control
	enable.add("GridFeatureCheck", boost::bind(&isGridFeatureEnabled, _2));
	commit.add("OpenGridStatus", boost::bind(&openGridStatus)); // <FS:Ansariel> FIRE-21236 - Help Menu - Check Grid Status doesn't open using External Browser

	// Agent
	commit.add("Agent.toggleFlying", boost::bind(&LLAgent::toggleFlying));
	enable.add("Agent.enableFlying", boost::bind(&LLAgent::enableFlying));
	commit.add("Agent.PressMicrophone", boost::bind(&LLAgent::pressMicrophone, _2));
	commit.add("Agent.ReleaseMicrophone", boost::bind(&LLAgent::releaseMicrophone, _2));
	commit.add("Agent.ToggleMicrophone", boost::bind(&LLAgent::toggleMicrophone, _2));
	enable.add("Agent.IsMicrophoneOn", boost::bind(&LLAgent::isMicrophoneOn, _2));
	enable.add("Agent.IsActionAllowed", boost::bind(&LLAgent::isActionAllowed, _2));

	// File menu
	init_menu_file();

	view_listener_t::addMenu(new LLEditEnableTakeOff(), "Edit.EnableTakeOff");
	view_listener_t::addMenu(new LLEditEnableCustomizeAvatar(), "Edit.EnableCustomizeAvatar");
	view_listener_t::addMenu(new LLEnableEditShape(), "Edit.EnableEditShape");
	view_listener_t::addMenu(new LLEnableHoverHeight(), "Edit.EnableHoverHeight");
	view_listener_t::addMenu(new LLEnableEditPhysics(), "Edit.EnableEditPhysics");
	commit.add("CustomizeAvatar", boost::bind(&handle_customize_avatar));
	commit.add("EditOutfit", boost::bind(&handle_edit_outfit));
	commit.add("EditShape", boost::bind(&handle_edit_shape));
	commit.add("HoverHeight", boost::bind(&handle_hover_height));
	commit.add("EditPhysics", boost::bind(&handle_edit_physics));
	// <FS:TT> Client LSL Bridge
	commit.add("RecreateLSLBridge", boost::bind(&handle_recreate_lsl_bridge));
	// </FS:TT>

	// View menu
	view_listener_t::addMenu(new LLViewMouselook(), "View.Mouselook");
	view_listener_t::addMenu(new LLViewJoystickFlycam(), "View.JoystickFlycam");
	view_listener_t::addMenu(new LLViewResetView(), "View.ResetView");
	view_listener_t::addMenu(new LLViewLookAtLastChatter(), "View.LookAtLastChatter");
	view_listener_t::addMenu(new LLViewShowHoverTips(), "View.ShowHoverTips");
	view_listener_t::addMenu(new LLViewHighlightTransparent(), "View.HighlightTransparent");
	view_listener_t::addMenu(new LLViewToggleRenderType(), "View.ToggleRenderType");
	view_listener_t::addMenu(new LLViewShowHUDAttachments(), "View.ShowHUDAttachments");
	view_listener_t::addMenu(new LLZoomer(1.2f), "View.ZoomOut");
	view_listener_t::addMenu(new LLZoomer(1/1.2f), "View.ZoomIn");
	view_listener_t::addMenu(new LLZoomer(DEFAULT_FIELD_OF_VIEW, false), "View.ZoomDefault");
	view_listener_t::addMenu(new LLViewDefaultUISize(), "View.DefaultUISize");
	view_listener_t::addMenu(new LLViewToggleUI(), "View.ToggleUI");
	view_listener_t::addMenu(new LLViewCheckToggleUI(), "View.CheckToggleUI"); // <FS:Ansariel> Notification not showing if hiding the UI

	view_listener_t::addMenu(new LLViewEnableMouselook(), "View.EnableMouselook");
	view_listener_t::addMenu(new LLViewEnableJoystickFlycam(), "View.EnableJoystickFlycam");
	view_listener_t::addMenu(new LLViewEnableLastChatter(), "View.EnableLastChatter");

	view_listener_t::addMenu(new LLViewCheckJoystickFlycam(), "View.CheckJoystickFlycam");
	view_listener_t::addMenu(new LLViewCheckShowHoverTips(), "View.CheckShowHoverTips");
	view_listener_t::addMenu(new LLViewCheckHighlightTransparent(), "View.CheckHighlightTransparent");
	view_listener_t::addMenu(new LLViewCheckRenderType(), "View.CheckRenderType");
	view_listener_t::addMenu(new LLViewStatusAway(), "View.Status.CheckAway");
	view_listener_t::addMenu(new LLViewStatusDoNotDisturb(), "View.Status.CheckDoNotDisturb");
	view_listener_t::addMenu(new LLViewCheckHUDAttachments(), "View.CheckHUDAttachments");
	enable.add("View.EnableHUDAttachments", boost::bind(&enable_show_HUD_attachments)); // <FS:Ansariel> Disable Show HUD attachments if prevented by RLVa
	// <FS:Zi> Add reset camera angles menu
	view_listener_t::addMenu(new LLViewResetCameraAngles(), "View.ResetCameraAngles");
	// </FS:Zi>
	
	// Me > Movement
	view_listener_t::addMenu(new LLAdvancedAgentFlyingInfo(), "Agent.getFlying");

	//Communicate Nearby chat
	// <FS:Ansariel> [FS Communication UI]
	//view_listener_t::addMenu(new LLCommunicateNearbyChat(), "Communicate.NearbyChat");

	// Communicate > Voice morphing > Subscribe...
	commit.add("Communicate.VoiceMorphing.Subscribe", boost::bind(&handle_voice_morphing_subscribe));
	// Communicate > Voice morphing > Premium perk...
	commit.add("Communicate.VoiceMorphing.PremiumPerk", boost::bind(&handle_premium_voice_morphing_subscribe));
	LLVivoxVoiceClient * voice_clientp = LLVivoxVoiceClient::getInstance();
	enable.add("Communicate.VoiceMorphing.NoVoiceMorphing.Check"
		, boost::bind(&LLVivoxVoiceClient::onCheckVoiceEffect, voice_clientp, "NoVoiceMorphing"));
	commit.add("Communicate.VoiceMorphing.NoVoiceMorphing.Click"
		, boost::bind(&LLVivoxVoiceClient::onClickVoiceEffect, voice_clientp, "NoVoiceMorphing"));

	// World menu
	view_listener_t::addMenu(new LLWorldAlwaysRun(), "World.AlwaysRun");
	view_listener_t::addMenu(new LLWorldCreateLandmark(), "World.CreateLandmark");
	view_listener_t::addMenu(new LLWorldPlaceProfile(), "World.PlaceProfile");
	view_listener_t::addMenu(new LLWorldSetHomeLocation(), "World.SetHomeLocation");
	view_listener_t::addMenu(new LLWorldTeleportHome(), "World.TeleportHome");
	view_listener_t::addMenu(new LLWorldSetAway(), "World.SetAway");
	view_listener_t::addMenu(new LLWorldSetDoNotDisturb(), "World.SetDoNotDisturb");
	view_listener_t::addMenu(new LLWorldGetAway(), "World.GetAway"); //[SJ FIRE-2177]
	view_listener_t::addMenu(new LLWorldGetBusy(), "World.GetBusy"); //[SJ FIRE-2177]
	view_listener_t::addMenu(new LLWorldSetAutorespond(), "World.SetAutorespond");
	view_listener_t::addMenu(new LLWorldGetAutorespond(), "World.GetAutorespond");  //[SJ FIRE-2177]
	// <FS:PP> FIRE-1245: Option to block/reject teleport requests
	view_listener_t::addMenu(new LLWorldSetRejectTeleportOffers(), "World.SetRejectTeleportOffers");
	view_listener_t::addMenu(new LLWorldGetRejectTeleportOffers(), "World.GetRejectTeleportOffers");
	// </FS:PP>
	// <FS:PP> FIRE-15233: Automatic friendship request refusal
	view_listener_t::addMenu(new LLWorldSetRejectFriendshipRequests(), "World.SetRejectFriendshipRequests");
	view_listener_t::addMenu(new LLWorldGetRejectFriendshipRequests(), "World.GetRejectFriendshipRequests");
	// </FS:PP>
	// <FS:PP> FIRE-1245: Option to block/reject teleport requests
	view_listener_t::addMenu(new LLWorldSetRejectAllGroupInvites(), "World.SetRejectAllGroupInvites");
	view_listener_t::addMenu(new LLWorldGetRejectAllGroupInvites(), "World.GetRejectAllGroupInvites");
	// </FS:PP>
	view_listener_t::addMenu(new LLWorldSetAutorespondNonFriends(), "World.SetAutorespondNonFriends");
	view_listener_t::addMenu(new LLWorldGetAutorespondNonFriends(), "World.GetAutorespondNonFriends");  //[SJ FIRE-2177]
	view_listener_t::addMenu(new LLWorldEnableCreateLandmark(), "World.EnableCreateLandmark");
// [RLVa:KB]
	enable.add("World.EnablePlaceProfile", boost::bind(&enable_place_profile));
// [/RLVa:KB]
	view_listener_t::addMenu(new LLWorldEnableSetHomeLocation(), "World.EnableSetHomeLocation");
	view_listener_t::addMenu(new LLWorldEnableTeleportHome(), "World.EnableTeleportHome");
	view_listener_t::addMenu(new LLWorldEnableBuyLand(), "World.EnableBuyLand");

	view_listener_t::addMenu(new LLWorldCheckAlwaysRun(), "World.CheckAlwaysRun");
	
	view_listener_t::addMenu(new LLWorldEnvSettings(), "World.EnvSettings");
	view_listener_t::addMenu(new LLWorldEnableEnvSettings(), "World.EnableEnvSettings");
	view_listener_t::addMenu(new LLWorldEnvPreset(), "World.EnvPreset");
	view_listener_t::addMenu(new LLWorldEnableEnvPreset(), "World.EnableEnvPreset");
	view_listener_t::addMenu(new LLWorldPostProcess(), "World.PostProcess");

	// Tools menu
	view_listener_t::addMenu(new LLToolsSelectTool(), "Tools.SelectTool");
	view_listener_t::addMenu(new LLToolsSelectOnlyMyObjects(), "Tools.SelectOnlyMyObjects");
	view_listener_t::addMenu(new LLToolsSelectOnlyMovableObjects(), "Tools.SelectOnlyMovableObjects");
	view_listener_t::addMenu(new LLToolsSelectBySurrounding(), "Tools.SelectBySurrounding");
	view_listener_t::addMenu(new LLToolsShowHiddenSelection(), "Tools.ShowHiddenSelection");
	view_listener_t::addMenu(new LLToolsShowSelectionLightRadius(), "Tools.ShowSelectionLightRadius");
	view_listener_t::addMenu(new LLToolsEditLinkedParts(), "Tools.EditLinkedParts");
	view_listener_t::addMenu(new LLToolsSnapObjectXY(), "Tools.SnapObjectXY");
	view_listener_t::addMenu(new LLToolsUseSelectionForGrid(), "Tools.UseSelectionForGrid");
	view_listener_t::addMenu(new LLToolsSelectNextPartFace(), "Tools.SelectNextPart");
	commit.add("Tools.Link", boost::bind(&LLSelectMgr::linkObjects, LLSelectMgr::getInstance()));
	commit.add("Tools.Unlink", boost::bind(&LLSelectMgr::unlinkObjects, LLSelectMgr::getInstance()));
	view_listener_t::addMenu(new LLToolsStopAllAnimations(), "Tools.StopAllAnimations");
	view_listener_t::addMenu(new LLToolsReleaseKeys(), "Tools.ReleaseKeys");
	view_listener_t::addMenu(new LLToolsEnableReleaseKeys(), "Tools.EnableReleaseKeys");	
	commit.add("Tools.LookAtSelection", boost::bind(&handle_look_at_selection, _2));
	commit.add("Tools.ScriptInfo",boost::bind(&handle_script_info));
	commit.add("Tools.BuyOrTake", boost::bind(&handle_buy_or_take));
	commit.add("Tools.TakeCopy", boost::bind(&handle_take_copy));
	view_listener_t::addMenu(new LLToolsSaveToObjectInventory(), "Tools.SaveToObjectInventory");
	view_listener_t::addMenu(new LLToolsSelectedScriptAction(), "Tools.SelectedScriptAction");
	view_listener_t::addMenu(new FSToolsResyncAnimations(), "Tools.ResyncAnimations");	// <FS:CR> Resync Animations
	view_listener_t::addMenu(new FSToolsUndeform(), "Tools.Undeform");	// <FS:CR> FIRE-4345: Undeform
	view_listener_t::addMenu(new FSDerenderAnimatedObjects(), "Tools.DerenderAnimatedObjects");	// <FS:Beq> Animesh Kill switch

	view_listener_t::addMenu(new LLToolsEnableToolNotPie(), "Tools.EnableToolNotPie");
	view_listener_t::addMenu(new LLToolsEnableSelectNextPart(), "Tools.EnableSelectNextPart");
	enable.add("Tools.EnableLink", boost::bind(&LLSelectMgr::enableLinkObjects, LLSelectMgr::getInstance()));
	enable.add("Tools.EnableUnlink", boost::bind(&LLSelectMgr::enableUnlinkObjects, LLSelectMgr::getInstance()));
	view_listener_t::addMenu(new LLToolsEnableBuyOrTake(), "Tools.EnableBuyOrTake");
	enable.add("Tools.EnableTakeCopy", boost::bind(&enable_object_take_copy));
	enable.add("Tools.VisibleBuyObject", boost::bind(&tools_visible_buy_object));
	enable.add("Tools.VisibleTakeObject", boost::bind(&tools_visible_take_object));
	view_listener_t::addMenu(new LLToolsEnableSaveToObjectInventory(), "Tools.EnableSaveToObjectInventory");

	view_listener_t::addMenu(new LLToolsEnablePathfinding(), "Tools.EnablePathfinding");
	view_listener_t::addMenu(new LLToolsEnablePathfindingView(), "Tools.EnablePathfindingView");
	view_listener_t::addMenu(new LLToolsDoPathfindingRebakeRegion(), "Tools.DoPathfindingRebakeRegion");
	view_listener_t::addMenu(new LLToolsEnablePathfindingRebakeRegion(), "Tools.EnablePathfindingRebakeRegion");

	// Help menu
	// most items use the ShowFloater method
	view_listener_t::addMenu(new LLToggleHowTo(), "Help.ToggleHowTo");
	enable.add("Help.HowToVisible", boost::bind(&enable_how_to_visible, _2));

	// Advanced menu
	view_listener_t::addMenu(new LLAdvancedToggleConsole(), "Advanced.ToggleConsole");
	view_listener_t::addMenu(new LLAdvancedCheckConsole(), "Advanced.CheckConsole");
	view_listener_t::addMenu(new LLAdvancedDumpInfoToConsole(), "Advanced.DumpInfoToConsole");

	// Advanced > HUD Info
	view_listener_t::addMenu(new LLAdvancedToggleHUDInfo(), "Advanced.ToggleHUDInfo");
	view_listener_t::addMenu(new LLAdvancedCheckHUDInfo(), "Advanced.CheckHUDInfo");

	// Advanced Other Settings	
	view_listener_t::addMenu(new LLAdvancedClearGroupCache(), "Advanced.ClearGroupCache");
	
	// Advanced > Render > Types
	view_listener_t::addMenu(new LLAdvancedToggleRenderType(), "Advanced.ToggleRenderType");
	view_listener_t::addMenu(new LLAdvancedCheckRenderType(), "Advanced.CheckRenderType");

	//// Advanced > Render > Features
	view_listener_t::addMenu(new LLAdvancedToggleFeature(), "Advanced.ToggleFeature");
	view_listener_t::addMenu(new LLAdvancedCheckFeature(), "Advanced.CheckFeature");

	view_listener_t::addMenu(new LLAdvancedCheckDisplayTextureDensity(), "Advanced.CheckDisplayTextureDensity");
	view_listener_t::addMenu(new LLAdvancedSetDisplayTextureDensity(), "Advanced.SetDisplayTextureDensity");

	// Advanced > Render > Info Displays
	view_listener_t::addMenu(new LLAdvancedToggleInfoDisplay(), "Advanced.ToggleInfoDisplay");
	view_listener_t::addMenu(new LLAdvancedCheckInfoDisplay(), "Advanced.CheckInfoDisplay");
	view_listener_t::addMenu(new LLAdvancedSelectedTextureInfo(), "Advanced.SelectedTextureInfo");
	commit.add("Advanced.SelectedMaterialInfo", boost::bind(&handle_selected_material_info));
	view_listener_t::addMenu(new LLAdvancedToggleWireframe(), "Advanced.ToggleWireframe");
	view_listener_t::addMenu(new LLAdvancedCheckWireframe(), "Advanced.CheckWireframe");
	// Develop > Render
	view_listener_t::addMenu(new LLAdvancedEnableObjectObjectOcclusion(), "Advanced.EnableObjectObjectOcclusion");
	view_listener_t::addMenu(new LLAdvancedEnableRenderFBO(), "Advanced.EnableRenderFBO");
	view_listener_t::addMenu(new LLAdvancedEnableRenderDeferred(), "Advanced.EnableRenderDeferred");
	view_listener_t::addMenu(new LLAdvancedEnableRenderDeferredOptions(), "Advanced.EnableRenderDeferredOptions");
	view_listener_t::addMenu(new LLAdvancedToggleRandomizeFramerate(), "Advanced.ToggleRandomizeFramerate");
	view_listener_t::addMenu(new LLAdvancedCheckRandomizeFramerate(), "Advanced.CheckRandomizeFramerate");
	view_listener_t::addMenu(new LLAdvancedTogglePeriodicSlowFrame(), "Advanced.TogglePeriodicSlowFrame");
	view_listener_t::addMenu(new LLAdvancedCheckPeriodicSlowFrame(), "Advanced.CheckPeriodicSlowFrame");
	view_listener_t::addMenu(new LLAdvancedToggleFrameTest(), "Advanced.ToggleFrameTest");
	view_listener_t::addMenu(new LLAdvancedCheckFrameTest(), "Advanced.CheckFrameTest");
	view_listener_t::addMenu(new LLAdvancedHandleAttachedLightParticles(), "Advanced.HandleAttachedLightParticles");
	view_listener_t::addMenu(new LLAdvancedCheckRenderShadowOption(), "Advanced.CheckRenderShadowOption");
	view_listener_t::addMenu(new LLAdvancedClickRenderShadowOption(), "Advanced.ClickRenderShadowOption");
	view_listener_t::addMenu(new LLAdvancedClickRenderProfile(), "Advanced.ClickRenderProfile");
	view_listener_t::addMenu(new LLAdvancedClickRenderBenchmark(), "Advanced.ClickRenderBenchmark");
	//[FIX FIRE-1927 - enable DoubleClickTeleport shortcut : SJ]
	view_listener_t::addMenu(new LLAdvancedToggleDoubleClickTeleport, "Advanced.ToggleDoubleClickTeleport");

	#ifdef TOGGLE_HACKED_GODLIKE_VIEWER
	view_listener_t::addMenu(new LLAdvancedHandleToggleHackedGodmode(), "Advanced.HandleToggleHackedGodmode");
	view_listener_t::addMenu(new LLAdvancedCheckToggleHackedGodmode(), "Advanced.CheckToggleHackedGodmode");
	view_listener_t::addMenu(new LLAdvancedEnableToggleHackedGodmode(), "Advanced.EnableToggleHackedGodmode");
	#endif

	// Advanced > World
	view_listener_t::addMenu(new LLAdvancedDumpScriptedCamera(), "Advanced.DumpScriptedCamera");
	view_listener_t::addMenu(new LLAdvancedDumpRegionObjectCache(), "Advanced.DumpRegionObjectCache");

	// Advanced > UI
	commit.add("Advanced.WebBrowserTest", boost::bind(&handle_web_browser_test,	_2));	// sigh! this one opens the MEDIA browser
	commit.add("Advanced.WebContentTest", boost::bind(&handle_web_content_test, _2));	// this one opens the Web Content floater
	commit.add("Advanced.ShowURL", boost::bind(&handle_show_url, _2));
	commit.add("Advanced.ReportBug", boost::bind(&handle_report_bug, _2));
	view_listener_t::addMenu(new LLAdvancedBuyCurrencyTest(), "Advanced.BuyCurrencyTest");
	view_listener_t::addMenu(new LLAdvancedDumpSelectMgr(), "Advanced.DumpSelectMgr");
	view_listener_t::addMenu(new LLAdvancedDumpInventory(), "Advanced.DumpInventory");
	commit.add("Advanced.DumpTimers", boost::bind(&handle_dump_timers) );
	commit.add("Advanced.DumpFocusHolder", boost::bind(&handle_dump_focus) );
	view_listener_t::addMenu(new LLAdvancedPrintSelectedObjectInfo(), "Advanced.PrintSelectedObjectInfo");
	view_listener_t::addMenu(new LLAdvancedPrintAgentInfo(), "Advanced.PrintAgentInfo");
	view_listener_t::addMenu(new LLAdvancedToggleDebugClicks(), "Advanced.ToggleDebugClicks");
	view_listener_t::addMenu(new LLAdvancedCheckDebugClicks(), "Advanced.CheckDebugClicks");
	view_listener_t::addMenu(new LLAdvancedCheckDebugViews(), "Advanced.CheckDebugViews");
	view_listener_t::addMenu(new LLAdvancedToggleDebugViews(), "Advanced.ToggleDebugViews");
	view_listener_t::addMenu(new LLAdvancedToggleXUINameTooltips(), "Advanced.ToggleXUINameTooltips");
	view_listener_t::addMenu(new LLAdvancedCheckXUINameTooltips(), "Advanced.CheckXUINameTooltips");
	view_listener_t::addMenu(new LLAdvancedToggleDebugMouseEvents(), "Advanced.ToggleDebugMouseEvents");
	view_listener_t::addMenu(new LLAdvancedCheckDebugMouseEvents(), "Advanced.CheckDebugMouseEvents");
	view_listener_t::addMenu(new LLAdvancedToggleDebugKeys(), "Advanced.ToggleDebugKeys");
	view_listener_t::addMenu(new LLAdvancedCheckDebugKeys(), "Advanced.CheckDebugKeys");
	view_listener_t::addMenu(new LLAdvancedToggleDebugWindowProc(), "Advanced.ToggleDebugWindowProc");
	view_listener_t::addMenu(new LLAdvancedCheckDebugWindowProc(), "Advanced.CheckDebugWindowProc");

	// Advanced > XUI
	commit.add("Advanced.ReloadColorSettings", boost::bind(&LLUIColorTable::loadFromSettings, LLUIColorTable::getInstance()));
	view_listener_t::addMenu(new LLAdvancedLoadUIFromXML(), "Advanced.LoadUIFromXML");
	view_listener_t::addMenu(new LLAdvancedSaveUIToXML(), "Advanced.SaveUIToXML");
	view_listener_t::addMenu(new LLAdvancedToggleXUINames(), "Advanced.ToggleXUINames");
	view_listener_t::addMenu(new LLAdvancedCheckXUINames(), "Advanced.CheckXUINames");
	view_listener_t::addMenu(new LLAdvancedSendTestIms(), "Advanced.SendTestIMs");
	commit.add("Advanced.FlushNameCaches", boost::bind(&handle_flush_name_caches));

	// Advanced > Character > Grab Baked Texture
	view_listener_t::addMenu(new LLAdvancedGrabBakedTexture(), "Advanced.GrabBakedTexture");
	view_listener_t::addMenu(new LLAdvancedEnableGrabBakedTexture(), "Advanced.EnableGrabBakedTexture");

	// Advanced > Character > Character Tests
	view_listener_t::addMenu(new LLAdvancedAppearanceToXML(), "Advanced.AppearanceToXML");
	view_listener_t::addMenu(new LLAdvancedEnableAppearanceToXML(), "Advanced.EnableAppearanceToXML");
	view_listener_t::addMenu(new LLAdvancedToggleCharacterGeometry(), "Advanced.ToggleCharacterGeometry");

	view_listener_t::addMenu(new LLAdvancedTestMale(), "Advanced.TestMale");
	view_listener_t::addMenu(new LLAdvancedTestFemale(), "Advanced.TestFemale");
	
	// Advanced > Character > Animation Speed
	view_listener_t::addMenu(new LLAdvancedAnimTenFaster(), "Advanced.AnimTenFaster");
	view_listener_t::addMenu(new LLAdvancedAnimTenSlower(), "Advanced.AnimTenSlower");
	view_listener_t::addMenu(new LLAdvancedAnimResetAll(), "Advanced.AnimResetAll");

	// Advanced > Character (toplevel)
	view_listener_t::addMenu(new LLAdvancedForceParamsToDefault(), "Advanced.ForceParamsToDefault");
	view_listener_t::addMenu(new LLAdvancedReloadVertexShader(), "Advanced.ReloadVertexShader");
	view_listener_t::addMenu(new LLAdvancedToggleAnimationInfo(), "Advanced.ToggleAnimationInfo");
	view_listener_t::addMenu(new LLAdvancedCheckAnimationInfo(), "Advanced.CheckAnimationInfo");
	view_listener_t::addMenu(new LLAdvancedToggleShowLookAt(), "Advanced.ToggleShowLookAt");
	view_listener_t::addMenu(new LLAdvancedToggleShowColor(), "Advanced.ToggleShowColor");
	view_listener_t::addMenu(new LLAdvancedCheckShowColor(), "Advanced.CheckShowColor");
	view_listener_t::addMenu(new LLAdvancedCheckShowLookAt(), "Advanced.CheckShowLookAt");
	view_listener_t::addMenu(new LLAdvancedToggleShowPointAt(), "Advanced.ToggleShowPointAt");
	view_listener_t::addMenu(new LLAdvancedCheckShowPointAt(), "Advanced.CheckShowPointAt");
	view_listener_t::addMenu(new LLAdvancedTogglePrivateLookPointAt(), "Advanced.TogglePrivateLookPointAt");
	view_listener_t::addMenu(new LLAdvancedCheckPrivateLookPointAt(), "Advanced.CheckPrivateLookPointAt");
	view_listener_t::addMenu(new LLAdvancedToggleDebugJointUpdates(), "Advanced.ToggleDebugJointUpdates");
	view_listener_t::addMenu(new LLAdvancedCheckDebugJointUpdates(), "Advanced.CheckDebugJointUpdates");
	view_listener_t::addMenu(new LLAdvancedToggleDisableLOD(), "Advanced.ToggleDisableLOD");
	view_listener_t::addMenu(new LLAdvancedCheckDisableLOD(), "Advanced.CheckDisableLOD");
	view_listener_t::addMenu(new LLAdvancedToggleDebugCharacterVis(), "Advanced.ToggleDebugCharacterVis");
	view_listener_t::addMenu(new LLAdvancedCheckDebugCharacterVis(), "Advanced.CheckDebugCharacterVis");
	view_listener_t::addMenu(new LLAdvancedDumpAttachments(), "Advanced.DumpAttachments");
	view_listener_t::addMenu(new LLAdvancedRebakeTextures(), "Advanced.RebakeTextures");
// [SL:KB] - Patch: Appearance-PhantomAttach | Checked: Catznip-5.0
	commit.add("Advanced.RefreshAttachments", boost::bind(&handle_refresh_attachments));
// [/SL:KB]
	view_listener_t::addMenu(new LLAdvancedDebugAvatarTextures(), "Advanced.DebugAvatarTextures");
	view_listener_t::addMenu(new LLAdvancedDumpAvatarLocalTextures(), "Advanced.DumpAvatarLocalTextures");
	view_listener_t::addMenu(new LLAdvancedReloadAvatarCloudParticle(), "Advanced.ReloadAvatarCloudParticle");

	// Advanced > Network
	view_listener_t::addMenu(new LLAdvancedEnableMessageLog(), "Advanced.EnableMessageLog");
	view_listener_t::addMenu(new LLAdvancedDisableMessageLog(), "Advanced.DisableMessageLog");
	view_listener_t::addMenu(new LLAdvancedDropPacket(), "Advanced.DropPacket");

	// Advanced > Recorder
	view_listener_t::addMenu(new LLAdvancedAgentPilot(), "Advanced.AgentPilot");
	view_listener_t::addMenu(new LLAdvancedToggleAgentPilotLoop(), "Advanced.ToggleAgentPilotLoop");
	view_listener_t::addMenu(new LLAdvancedCheckAgentPilotLoop(), "Advanced.CheckAgentPilotLoop");
	view_listener_t::addMenu(new LLAdvancedViewerEventRecorder(), "Advanced.EventRecorder");

	// Advanced > Debugging
	view_listener_t::addMenu(new LLAdvancedForceErrorBreakpoint(), "Advanced.ForceErrorBreakpoint");
	view_listener_t::addMenu(new LLAdvancedForceErrorLlerror(), "Advanced.ForceErrorLlerror");
	view_listener_t::addMenu(new LLAdvancedForceErrorBadMemoryAccess(), "Advanced.ForceErrorBadMemoryAccess");
	view_listener_t::addMenu(new LLAdvancedForceErrorInfiniteLoop(), "Advanced.ForceErrorInfiniteLoop");
	view_listener_t::addMenu(new LLAdvancedForceErrorSoftwareException(), "Advanced.ForceErrorSoftwareException");
	view_listener_t::addMenu(new LLAdvancedForceErrorDriverCrash(), "Advanced.ForceErrorDriverCrash");
	view_listener_t::addMenu(new LLAdvancedForceErrorDisconnectViewer(), "Advanced.ForceErrorDisconnectViewer");

	// Advanced (toplevel)
	view_listener_t::addMenu(new LLAdvancedToggleShowObjectUpdates(), "Advanced.ToggleShowObjectUpdates");
	view_listener_t::addMenu(new LLAdvancedCheckShowObjectUpdates(), "Advanced.CheckShowObjectUpdates");
	view_listener_t::addMenu(new LLAdvancedCompressImage(), "Advanced.CompressImage");
	view_listener_t::addMenu(new LLAdvancedShowDebugSettings(), "Advanced.ShowDebugSettings");
	view_listener_t::addMenu(new LLAdvancedEnableViewAdminOptions(), "Advanced.EnableViewAdminOptions");
	view_listener_t::addMenu(new LLAdvancedToggleViewAdminOptions(), "Advanced.ToggleViewAdminOptions");
	view_listener_t::addMenu(new LLAdvancedCheckViewAdminOptions(), "Advanced.CheckViewAdminOptions");
	view_listener_t::addMenu(new LLAdvancedToggleVisualLeakDetector(), "Advanced.ToggleVisualLeakDetector");

	view_listener_t::addMenu(new LLAdvancedRequestAdminStatus(), "Advanced.RequestAdminStatus");
	view_listener_t::addMenu(new LLAdvancedLeaveAdminStatus(), "Advanced.LeaveAdminStatus");

	// Develop >Set logging level
	view_listener_t::addMenu(new LLDevelopCheckLoggingLevel(), "Develop.CheckLoggingLevel");
	view_listener_t::addMenu(new LLDevelopSetLoggingLevel(), "Develop.SetLoggingLevel");
	
	//Develop (Texture Fetch Debug Console)
	view_listener_t::addMenu(new LLDevelopTextureFetchDebugger(), "Develop.SetTexFetchDebugger");
	//Develop (clear cache immediately)
	commit.add("Develop.ClearCache", boost::bind(&handle_cache_clear_immediately) );

	// Admin >Object
	view_listener_t::addMenu(new LLAdminForceTakeCopy(), "Admin.ForceTakeCopy");
	view_listener_t::addMenu(new LLAdminHandleObjectOwnerSelf(), "Admin.HandleObjectOwnerSelf");
	view_listener_t::addMenu(new LLAdminHandleObjectOwnerPermissive(), "Admin.HandleObjectOwnerPermissive");
	view_listener_t::addMenu(new LLAdminHandleForceDelete(), "Admin.HandleForceDelete");
	view_listener_t::addMenu(new LLAdminHandleObjectLock(), "Admin.HandleObjectLock");
	view_listener_t::addMenu(new LLAdminHandleObjectAssetIDs(), "Admin.HandleObjectAssetIDs");

	// Admin >Parcel 
	view_listener_t::addMenu(new LLAdminHandleForceParcelOwnerToMe(), "Admin.HandleForceParcelOwnerToMe");
	view_listener_t::addMenu(new LLAdminHandleForceParcelToContent(), "Admin.HandleForceParcelToContent");
	view_listener_t::addMenu(new LLAdminHandleClaimPublicLand(), "Admin.HandleClaimPublicLand");

	// Admin >Region
	view_listener_t::addMenu(new LLAdminHandleRegionDumpTempAssetData(), "Admin.HandleRegionDumpTempAssetData");
	// Admin top level
	view_listener_t::addMenu(new LLAdminOnSaveState(), "Admin.OnSaveState");

	// Self context menu
	view_listener_t::addMenu(new LLSelfStandUp(), "Self.StandUp");
	enable.add("Self.EnableStandUp", boost::bind(&enable_standup_self));
	view_listener_t::addMenu(new LLSelfSitDown(), "Self.SitDown");
	enable.add("Self.EnableSitDown", boost::bind(&enable_sitdown_self)); 
	enable.add("Self.ShowSitDown", boost::bind(&show_sitdown_self));
	view_listener_t::addMenu(new FSSelfForceSit(), "Self.ForceSit"); //KC
	enable.add("Self.EnableForceSit", boost::bind(&enable_forcesit_self)); //KC
	view_listener_t::addMenu(new FSSelfCheckForceSit(), "Self.getForceSit"); //KC
	view_listener_t::addMenu(new FSSelfToggleMoveLock(), "Self.ToggleMoveLock"); //KC
	view_listener_t::addMenu(new FSSelfCheckMoveLock(), "Self.GetMoveLock"); //KC
	enable.add("Self.EnableMoveLock", boost::bind(&enable_move_lock));	// <FS:CR>
	view_listener_t::addMenu(new FSSelfToggleIgnorePreJump(), "Self.toggleIgnorePreJump"); //SJ
	view_listener_t::addMenu(new FSSelfCheckIgnorePreJump(), "Self.getIgnorePreJump"); //SJ
	view_listener_t::addMenu(new LLSelfRemoveAllAttachments(), "Self.RemoveAllAttachments");

	view_listener_t::addMenu(new LLSelfEnableRemoveAllAttachments(), "Self.EnableRemoveAllAttachments");

	// we don't use boost::bind directly to delay side tray construction
	view_listener_t::addMenu( new LLTogglePanelPeopleTab(), "SideTray.PanelPeopleTab");
	view_listener_t::addMenu( new LLCheckPanelPeopleTab(), "SideTray.CheckPanelPeopleTab");

	 // Avatar pie menu
	view_listener_t::addMenu(new LLAvatarCheckImpostorMode(), "Avatar.CheckImpostorMode");
	view_listener_t::addMenu(new LLAvatarSetImpostorMode(), "Avatar.SetImpostorMode");
	view_listener_t::addMenu(new LLObjectMute(), "Avatar.Mute");
	view_listener_t::addMenu(new LLAvatarAddFriend(), "Avatar.AddFriend");
	view_listener_t::addMenu(new LLAvatarAddContact(), "Avatar.AddContact");
	commit.add("Avatar.Freeze", boost::bind(&handle_avatar_freeze, LLSD()));
	view_listener_t::addMenu(new LLAvatarDebug(), "Avatar.Debug");
	view_listener_t::addMenu(new LLAvatarVisibleDebug(), "Avatar.VisibleDebug");
	view_listener_t::addMenu(new LLAvatarInviteToGroup(), "Avatar.InviteToGroup");
	// <FS:Ansariel> FIRE-13515: Re-add give calling card
	view_listener_t::addMenu(new LLAvatarGiveCard(), "Avatar.GiveCard");
	// </FS:Ansariel> FIRE-13515: Re-add give calling card
	commit.add("Avatar.Eject", boost::bind(&handle_avatar_eject, LLSD()));
	commit.add("Avatar.ShowInspector", boost::bind(&handle_avatar_show_inspector));
	view_listener_t::addMenu(new LLAvatarSendIM(), "Avatar.SendIM");
	view_listener_t::addMenu(new LLAvatarCall(), "Avatar.Call");
//	enable.add("Avatar.EnableCall", boost::bind(&LLAvatarActions::canCall));
// [RLVa:KB] - Checked: 2010-08-25 (RLVa-1.2.1b) | Added: RLVa-1.2.1b
	enable.add("Avatar.EnableCall", boost::bind(&enable_avatar_call));
// [/RLVa:KB]
	view_listener_t::addMenu(new LLAvatarReportAbuse(), "Avatar.ReportAbuse");
	view_listener_t::addMenu(new LLAvatarTexRefresh(), "Avatar.TexRefresh");	// ## Zi: Texture Refresh

	view_listener_t::addMenu(new LLAvatarToggleMyProfile(), "Avatar.ToggleMyProfile");
	view_listener_t::addMenu(new LLAvatarResetSkeleton(), "Avatar.ResetSkeleton");
	view_listener_t::addMenu(new LLAvatarEnableResetSkeleton(), "Avatar.EnableResetSkeleton");
	view_listener_t::addMenu(new LLAvatarResetSkeletonAndAnimations(), "Avatar.ResetSkeletonAndAnimations");
	enable.add("Avatar.IsMyProfileOpen", boost::bind(&my_profile_visible));

	commit.add("Avatar.OpenMarketplace", boost::bind(&LLWeb::loadURLExternal, gSavedSettings.getString("MarketplaceURL")));
	
	view_listener_t::addMenu(new LLAvatarEnableAddFriend(), "Avatar.EnableAddFriend");
	enable.add("Avatar.EnableFreezeEject", boost::bind(&enable_freeze_eject, _2));

	// Object pie menu
	view_listener_t::addMenu(new LLObjectBuild(), "Object.Build");
	commit.add("Object.Touch", boost::bind(&handle_object_touch));
	commit.add("Object.SitOrStand", boost::bind(&handle_object_sit_or_stand));
	commit.add("Object.Delete", boost::bind(&handle_object_delete));
	view_listener_t::addMenu(new LLObjectAttachToAvatar(true), "Object.AttachToAvatar");
	view_listener_t::addMenu(new LLObjectAttachToAvatar(false), "Object.AttachAddToAvatar");
	view_listener_t::addMenu(new LLObjectReturn(), "Object.Return");
	commit.add("Object.Duplicate", boost::bind(&LLSelectMgr::duplicate, LLSelectMgr::getInstance()));
	view_listener_t::addMenu(new LLObjectReportAbuse(), "Object.ReportAbuse");
	view_listener_t::addMenu(new LLObjectMute(), "Object.Mute");
	view_listener_t::addMenu(new LLObjectDerender(), "Object.Derender");
	view_listener_t::addMenu(new LLObjectDerenderPermanent(), "Object.DerenderPermanent"); // <FS:Ansariel> Optional derender & blacklist
	enable.add("Object.EnableDerender", boost::bind(&enable_derender_object));	// <FS:CR> FIRE-10082 - Don't enable derendering own attachments when RLVa is enabled as well
	view_listener_t::addMenu(new LLObjectTexRefresh(), "Object.TexRefresh");	// ## Zi: Texture Refresh
	view_listener_t::addMenu(new LLEditParticleSource(), "Object.EditParticles");
   	view_listener_t::addMenu(new LLEnableEditParticleSource(), "Object.EnableEditParticles");

	enable.add("Object.VisibleTake", boost::bind(&visible_take_object));
	enable.add("Object.VisibleBuy", boost::bind(&visible_buy_object));

	commit.add("Object.Buy", boost::bind(&handle_buy));
	commit.add("Object.Edit", boost::bind(&handle_object_edit));
	commit.add("Object.Inspect", boost::bind(&handle_object_inspect));
	commit.add("Object.Open", boost::bind(&handle_object_open));
	commit.add("Object.Take", boost::bind(&handle_take));
	commit.add("Object.ShowInspector", boost::bind(&handle_object_show_inspector));
	enable.add("Object.EnableOpen", boost::bind(&enable_object_open));
	enable.add("Object.EnableTouch", boost::bind(&enable_object_touch, _1));
	enable.add("Object.EnableDelete", boost::bind(&enable_object_delete));
//	enable.add("Object.EnableWear", boost::bind(&object_selected_and_point_valid));
// [RLVa:KB] - Checked: 2010-03-16 (RLVa-1.2.0a) | Added: RLVa-1.2.0a
	enable.add("Object.EnableWear", boost::bind(&object_selected_and_point_valid, _2));
// [/RLVa:KB]

	enable.add("Object.EnableStandUp", boost::bind(&enable_object_stand_up));
	enable.add("Object.EnableSit", boost::bind(&enable_object_sit, _1));

	view_listener_t::addMenu(new LLObjectEnableReturn(), "Object.EnableReturn");
	enable.add("Object.EnableDuplicate", boost::bind(&LLSelectMgr::canDuplicate, LLSelectMgr::getInstance()));
	view_listener_t::addMenu(new LLObjectEnableReportAbuse(), "Object.EnableReportAbuse");

	enable.add("Avatar.EnableMute", boost::bind(&enable_object_mute));
	enable.add("Object.EnableMute", boost::bind(&enable_object_mute));
	enable.add("Object.EnableUnmute", boost::bind(&enable_object_unmute));
	enable.add("Object.EnableBuy", boost::bind(&enable_buy_object));
	commit.add("Object.ZoomIn", boost::bind(&handle_look_at_selection, "zoom"));
	enable.add("Object.EnableScriptInfo", boost::bind(&enable_script_info));	// <FS:CR>

	// Attachment pie menu
	enable.add("Attachment.Label", boost::bind(&onEnableAttachmentLabel, _1, _2));
	view_listener_t::addMenu(new LLAttachmentDrop(), "Attachment.Drop");
	view_listener_t::addMenu(new LLAttachmentDetachFromPoint(), "Attachment.DetachFromPoint");
	view_listener_t::addMenu(new LLAttachmentDetach(), "Attachment.Detach");
	view_listener_t::addMenu(new LLAttachmentPointFilled(), "Attachment.PointFilled");
	view_listener_t::addMenu(new LLAttachmentEnableDrop(), "Attachment.EnableDrop");
	view_listener_t::addMenu(new LLAttachmentEnableDetach(), "Attachment.EnableDetach");

	// Land pie menu
	view_listener_t::addMenu(new LLLandBuild(), "Land.Build");
	view_listener_t::addMenu(new LLLandSit(), "Land.Sit");
	view_listener_t::addMenu(new LLLandBuyPass(), "Land.BuyPass");
	view_listener_t::addMenu(new LLLandEdit(), "Land.Edit");

	// Particle muting
	view_listener_t::addMenu(new LLMuteParticle(), "Particle.Mute");

	view_listener_t::addMenu(new LLLandEnableBuyPass(), "Land.EnableBuyPass");
	commit.add("Land.Buy", boost::bind(&handle_buy_land));

	// Generic actions
	commit.add("ReportAbuse", boost::bind(&handle_report_abuse));
	commit.add("BuyCurrency", boost::bind(&handle_buy_currency));
	view_listener_t::addMenu(new LLShowHelp(), "ShowHelp");
	view_listener_t::addMenu(new LLToggleHelp(), "ToggleHelp");
	view_listener_t::addMenu(new LLToggleSpeak(), "ToggleSpeak");
	view_listener_t::addMenu(new LLPromptShowURL(), "PromptShowURL");
	view_listener_t::addMenu(new LLShowAgentProfile(), "ShowAgentProfile");
	view_listener_t::addMenu(new LLToggleAgentProfile(), "ToggleAgentProfile");
	view_listener_t::addMenu(new LLToggleControl(), "ToggleControl");
	view_listener_t::addMenu(new LLCheckControl(), "CheckControl");
	view_listener_t::addMenu(new LLGoToObject(), "GoToObject");
	commit.add("PayObject", boost::bind(&handle_give_money_dialog));

	// <FS:Ansariel> Control enhancements
	view_listener_t::addMenu(new LLTogglePerAccountControl(), "TogglePerAccountControl");
	view_listener_t::addMenu(new LLCheckPerAccountControl(), "CheckPerAccountControl");
	view_listener_t::addMenu(new FSResetControl(), "ResetControl");
	view_listener_t::addMenu(new FSResetPerAccountControl(), "ResetPerAccountControl");
	// </FS:Ansariel> Control enhancements

	// <FS:Ansariel> Reset Mesh LOD
	view_listener_t::addMenu(new FSResetMeshLOD(), "Avatar.ResetMeshLOD");

	commit.add("Inventory.NewWindow", boost::bind(&LLPanelMainInventory::newWindow));

	enable.add("EnablePayObject", boost::bind(&enable_pay_object));
	enable.add("EnablePayAvatar", boost::bind(&enable_pay_avatar));
	enable.add("EnableEdit", boost::bind(&enable_object_edit));
	enable.add("EnableMuteParticle", boost::bind(&enable_mute_particle));
	enable.add("VisibleBuild", boost::bind(&enable_object_build));
	commit.add("Pathfinding.Linksets.Select", boost::bind(&LLFloaterPathfindingLinksets::openLinksetsWithSelectedObjects));
	enable.add("EnableSelectInPathfindingLinksets", boost::bind(&enable_object_select_in_pathfinding_linksets));
	enable.add("VisibleSelectInPathfindingLinksets", boost::bind(&visible_object_select_in_pathfinding_linksets));
	commit.add("Pathfinding.Characters.Select", boost::bind(&LLFloaterPathfindingCharacters::openCharactersWithSelectedObjects));
	enable.add("EnableSelectInPathfindingCharacters", boost::bind(&enable_object_select_in_pathfinding_characters));
	enable.add("EnableBridgeFunction", boost::bind(&enable_bridge_function));	// <FS:CR>

	view_listener_t::addMenu(new LLFloaterVisible(), "FloaterVisible");
	view_listener_t::addMenu(new LLSomethingSelected(), "SomethingSelected");
	view_listener_t::addMenu(new LLSomethingSelectedNoHUD(), "SomethingSelectedNoHUD");
	view_listener_t::addMenu(new LLEditableSelected(), "EditableSelected");
	view_listener_t::addMenu(new LLEditableSelectedMono(), "EditableSelectedMono");
	view_listener_t::addMenu(new LLToggleUIHints(), "ToggleUIHints");

// [RLVa:KB] - Checked: RLVa-2.0.0
	enable.add("RLV.MainToggleVisible", boost::bind(&rlvMenuMainToggleVisible, _1));
	//if (RlvActions::isRlvEnabled()) // <FS:Ansariel> FIRE-20539: Toolbar buttons don't show disabled state anymore
	{
		enable.add("RLV.CanShowName", boost::bind(&rlvMenuCanShowName));
		enable.add("RLV.EnableIfNot", boost::bind(&rlvMenuEnableIfNot, _2));
	}
// [/RLVa:KB]

	// <FS:Ansariel> Toggle internal web browser
	commit.add("ToggleWebBrowser", boost::bind(&toggleWebBrowser, _2));
	// <FS:Ansariel> Toggle debug settings floater
	commit.add("ToggleSettingsDebug", boost::bind(&toggleSettingsDebug));
	// <FS:Ansariel> Toggle teleport history panel directly
	commit.add("ToggleTeleportHistory", boost::bind(&toggleTeleportHistory));
	// <FS:Ansariel> FIRE-7758: Save/load camera position
	commit.add("Camera.StoreView", boost::bind(&LLAgentCamera::storeCameraPosition, &gAgentCamera));
	commit.add("Camera.LoadView", boost::bind(&LLAgentCamera::loadCameraPosition, &gAgentCamera));
	// </FS:Ansariel>

	// <FS:Ansariel> Script debug floater
	commit.add("ShowScriptDebug", boost::bind(&LLFloaterScriptDebug::show, LLUUID::null));
	
	// <FS:CR> Stream list import/export
	view_listener_t::addMenu(new FSStreamListExportXML(), "Streamlist.xml_export");
	view_listener_t::addMenu(new FSStreamListImportXML(), "Streamlist.xml_import");
	// <FS:CR> Dump SimulatorFeatures to chat
	view_listener_t::addMenu(new FSDumpSimulatorFeaturesToChat(), "Develop.DumpSimFeaturesToChat");
	// <FS:CR> Add to contact set
	view_listener_t::addMenu(new FSAddToContactSet(), "Avatar.AddToContactSet");

	// <FS:Techwolf Lupindo> export
	view_listener_t::addMenu(new FSObjectExport(), "Object.Export");
	view_listener_t::addMenu(new FSObjectExportCollada(), "Object.ExportCollada");
	enable.add("Object.EnableExport", boost::bind(&enable_export_object));
	// </FS:Techwolf Lupindo>
}

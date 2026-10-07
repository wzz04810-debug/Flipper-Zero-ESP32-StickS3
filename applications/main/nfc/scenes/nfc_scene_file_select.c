#include "../nfc_app_i.h"
#include "../helpers/protocol_support/nfc_protocol_support.h"

void nfc_scene_file_select_on_enter(void* context) {
    NfcApp* instance = context;

    if(nfc_load_from_file_select(instance)) {
        /* Match Bruce-style file workflow: loading a saved card only loads
         * the data. Show the operation menu instead of starting emulation
         * immediately, so the user can choose emulate, write, edit, info,
         * rename, or delete according to the loaded protocol. */
        const NfcProtocol protocol = nfc_device_get_protocol(instance->nfc_device);
        FURI_LOG_I(
            "NfcFileSelect",
            "Loaded %s protocol=%u; opening operation menu",
            furi_string_get_cstr(instance->file_name),
            protocol);
        scene_manager_next_scene(instance->scene_manager, NfcSceneSavedMenu);
    } else {
        scene_manager_previous_scene(instance->scene_manager);
    }
}

bool nfc_scene_file_select_on_event(void* context, SceneManagerEvent event) {
    UNUSED(context);
    UNUSED(event);
    bool consumed = false;
    return consumed;
}

void nfc_scene_file_select_on_exit(void* context) {
    UNUSED(context);
}

#include "register_types.h"

#include "src/eosg_active_session.h"
#include "src/eosg_continuance_token.h"
#include "src/eosg_file_transfer_request.h"
#include "src/eosg_lobby_details.h"
#include "src/eosg_lobby_modification.h"
#include "src/eosg_lobby_search.h"
#include "src/eosg_multiplayer_peer.h"
#include "src/eosg_packet_peer_mediator.h"
#include "src/eosg_presence_modification.h"
#include "src/eosg_session_details.h"
#include "src/eosg_session_modification.h"
#include "src/eosg_session_search.h"
#include "src/eosg_transaction.h"
#include "src/ieos.h"
#include "core/core_bind.h"

static IEOS *_ieos;
static EOSGPacketPeerMediator *_mediator;

void initialize_eosg_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    ClassDB::register_class<IEOS>();
    _ieos = memnew(IEOS);
    CoreBind::Engine::get_singleton()->register_singleton("IEOS", IEOS::get_singleton());

    ClassDB::register_class<EOSGPacketPeerMediator>();
    _mediator = memnew(EOSGPacketPeerMediator);
    CoreBind::Engine::get_singleton()->register_singleton("EOSGPacketPeerMediator", EOSGPacketPeerMediator::get_singleton());

    ClassDB::register_abstract_class<EOSGFileTransferRequest>();
    ClassDB::register_class<EOSGPlayerDataStorageFileTransferRequest>();
    ClassDB::register_class<EOSGTitleStorageFileTransferRequest>();

    ClassDB::register_class<EOSGActiveSession>();
    ClassDB::register_class<EOSGContinuanceToken>();
    ClassDB::register_class<EOSGLobbyDetails>();
    ClassDB::register_class<EOSGLobbyModification>();
    ClassDB::register_class<EOSGLobbySearch>();
    ClassDB::register_class<EOSGMultiplayerPeer>();
    ClassDB::register_class<EOSGPresenceModification>();
    ClassDB::register_class<EOSGSessionDetails>();
    ClassDB::register_class<EOSGSessionModification>();
    ClassDB::register_class<EOSGSessionSearch>();
    ClassDB::register_class<EOSGTransaction>();
}

void uninitialize_eosg_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    CoreBind::Engine::get_singleton()->unregister_singleton("EOSGPacketPeerMediator");
    CoreBind::Engine::get_singleton()->unregister_singleton("IEOS");

    memdelete(_mediator);
    memdelete(_ieos);

    _mediator = nullptr;
    _ieos = nullptr;
}

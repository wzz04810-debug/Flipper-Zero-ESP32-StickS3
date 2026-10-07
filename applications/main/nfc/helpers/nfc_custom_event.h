#pragma once

typedef enum {
    // Reserve first 100 events for button types and indexes, starting from 0
    NfcCustomEventReserved = 100,

    // Mf classic dict attack events
    NfcCustomEventDictAttackComplete,
    NfcCustomEventDictAttackSkip,
    NfcCustomEventDictAttackDataUpdate,

    NfcCustomEventCardDetected,
    NfcCustomEventCardLost,

    NfcCustomEventViewExit,
    NfcCustomEventRetry,
    NfcCustomEventWorkerExit,
    NfcCustomEventWorkerUpdate,
    NfcCustomEventWrongCard,
    NfcCustomEventTimerExpired,
    NfcCustomEventByteInputDone,
    NfcCustomEventTextInputDone,
    NfcCustomEventDictAttackDone,

    NfcCustomEventRpcLoadFile,
    NfcCustomEventRpcExit,
    NfcCustomEventRpcSessionClose,

    NfcCustomEventPollerSuccess,
    NfcCustomEventPollerIncomplete,
    NfcCustomEventPollerFailure,

    NfcCustomEventListenerUpdate,

    NfcCustomEventEmulationTimeExpired,

    // ChameleonUltra BLE backend
    NfcCustomEventChameleonButton,
    NfcCustomEventChameleonConnected,
    NfcCustomEventChameleonFailed,
    NfcCustomEventChameleonCardRead,

    // 克隆源卡读取完成后，延后一个事件循环再进入写卡页面，避免场景进入回调重入。
    NfcCustomEventCloneSourceReady,
} NfcCustomEvent;

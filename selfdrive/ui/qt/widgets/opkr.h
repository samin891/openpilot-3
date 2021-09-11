#pragma once

#include <QPushButton>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/ui.h"


class SshLegacyToggle : public ToggleControl {
  Q_OBJECT

public:
  SshLegacyToggle() : ToggleControl("기존 공개KEY 사용", "SSH 접속시 기존 공개KEY(0.8.2이하)를 사용합니다.", "", Params().getBool("OpkrSSHLegacy")) {
    QObject::connect(this, &SshLegacyToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("OpkrSSHLegacy", status);
    });
  }
};


class DebugUiOneToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiOneToggle() : ToggleControl("DEBUG UI 1", "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi1")) {
    QObject::connect(this, &DebugUiOneToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi1", status);
      if (state) {
        QUIState::ui_state.scene.nDebugUi1 = true;
      } else {
        QUIState::ui_state.scene.nDebugUi1 = false;
      }
    });
  }
};


class DebugUiTwoToggle : public ToggleControl {
  Q_OBJECT

public:
  DebugUiTwoToggle() : ToggleControl("DEBUG UI 2", "", "../assets/offroad/icon_shell.png", Params().getBool("DebugUi2")) {
    QObject::connect(this, &DebugUiTwoToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("DebugUi2", status);
      if (state) {
        QUIState::ui_state.scene.nDebugUi2 = true;
      } else {
        QUIState::ui_state.scene.nDebugUi2 = false;
      }
    });
  }
};


class PrebuiltToggle : public ToggleControl {
  Q_OBJECT

public:
  PrebuiltToggle() : ToggleControl("Prebuilt 파일 생성", "Prebuilt 파일을 생성하며 부팅속도를 단축시킵니다. UI수정을 한 경우 기능을 끄십시오.", "../assets/offroad/icon_shell.png", Params().getBool("PutPrebuiltOn")) {
    QObject::connect(this, &PrebuiltToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("PutPrebuiltOn", status);
    });
  }
};


class RadarLongHelperToggle : public ToggleControl {
  Q_OBJECT

public:
  RadarLongHelperToggle() : ToggleControl("레이더 롱 보조 사용", "비전 SCC 사용 중 근거리(25m이하)에서 레이더값+콤마비전롱(보간)을 사용합니다. 비전SCC가 충분히 멈추지 못하는 상황에서 레이더 값을 이용해 확실히 멈출 수 있게 합니다. 레이더가 앞차 인식시만 사용되며, 앞차인식을 못할 시(녹색쉐브론)는 콤마비전롱으로만 감속됩니다. 이 기능을 끄면 항상 콤마 비전롱을 사용하는것을 의미합니다.(레이더인식시 앞차거리 4m 이하는 안전을 위해 레이더값을 강제로 사용함)", "../assets/offroad/icon_shell.png", Params().getBool("RadarLongHelper")) {
    QObject::connect(this, &RadarLongHelperToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("RadarLongHelper", status);
    });
  }
};


class StoppingDistAdjToggle : public ToggleControl {
  Q_OBJECT

public:
  StoppingDistAdjToggle() : ToggleControl("정지거리 조정", "레이더 정지거리보다 조금 더 앞에 정지합니다. 일부 울컥거림 현상이 나타날 수 있으니 불편하신분들은 기능을 끄십시오.", "../assets/offroad/icon_shell.png", Params().getBool("StoppingDistAdj")) {
    QObject::connect(this, &StoppingDistAdjToggle::toggleFlipped, [=](int state) {
      bool status = state ? true : false;
      Params().putBool("StoppingDistAdj", status);
    });
  }
};


// 오픈파일럿 미리보기
class OpenpilotView : public AbstractControl {
  Q_OBJECT

public:
  OpenpilotView();

private:
  QPushButton btn;
  Params params;
  
  void refresh();
};


class GitHash : public AbstractControl {
  Q_OBJECT

public:
  GitHash();

private:
  QLabel local_hash;
  QLabel remote_hash;
  Params params;
};
#include "selfdrive/ui/qt/widgets/opkr.h"

#include <QHBoxLayout>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/widgets/input.h"

#include "selfdrive/ui/ui.h"


GitHash::GitHash() : AbstractControl("커밋(로컬/리모트)", "", "") {

  QString lhash = QString::fromStdString(params.get("GitCommit").substr(0, 10));
  QString rhash = QString::fromStdString(params.get("GitCommitRemote").substr(0, 10));
  hlayout->addStretch(1);
  
  local_hash.setText(QString::fromStdString(params.get("GitCommit").substr(0, 10)));
  remote_hash.setText(QString::fromStdString(params.get("GitCommitRemote").substr(0, 10)));
  remote_hash.setAlignment(Qt::AlignVCenter);
  local_hash.setStyleSheet("color: #aaaaaa");
  if (lhash == rhash) {
    remote_hash.setStyleSheet("color: #aaaaaa");
  } else {
    remote_hash.setStyleSheet("color: #0099ff");
  }
  hlayout->addWidget(&local_hash);
  hlayout->addWidget(&remote_hash);
}

OpenpilotView::OpenpilotView() : AbstractControl("오픈파일럿 주행화면 미리보기", "오픈파일럿 주행화면을 미리보기 합니다.", "") {

  // setup widget
  hlayout->addStretch(1);

  btn.setStyleSheet(R"(
    padding: 0;
    border-radius: 50px;
    font-size: 35px;
    font-weight: 500;
    color: #E4E4E4;
    background-color: #393939;
  )");

  btn.setFixedSize(250, 100);
  hlayout->addWidget(&btn);

  QObject::connect(&btn, &QPushButton::clicked, [=]() {
    bool stat = params.getBool("IsOpenpilotViewEnabled");
    if (stat) {
      params.putBool("IsOpenpilotViewEnabled", false);
    } else {
      params.putBool("IsOpenpilotViewEnabled", true);
    }
    refresh();
  });
  refresh();
}

void OpenpilotView::refresh() {
  bool param = params.getBool("IsOpenpilotViewEnabled");
  QString car_param = QString::fromStdString(params.get("CarParams"));
  if (param) {
    btn.setText("미리보기해제");
  } else {
    btn.setText("미리보기");
  }
  if (car_param.length()) {
    btn.setEnabled(false);
  } else {
    btn.setEnabled(true);
  }
}
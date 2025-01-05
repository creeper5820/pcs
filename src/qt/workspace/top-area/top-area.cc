#include "top-area.hh"

#include <creeper-qt/module/round-icon-button.hh>
#include <creeper-qt/widget/line-edit.hh>
#include <creeper-qt/widget/push-button.hh>

using namespace qt;
using namespace creeper;

struct TopArea::Impl {
    Impl() {
        main.setRadius(15);
        main.setIconRatio(1);
        main.setIcon(QIcon(":/pcs/pic/edit.png"));

        file.setText("文件");
        file.setFont(QFont("Nowar Warcraft Sans CN", 8));
        file.setFixedSize(45, 30);
        file.disableBackground();

        edit.setText("编辑");
        edit.setFont(QFont("Nowar Warcraft Sans CN", 8));
        edit.setFixedSize(45, 30);
        edit.disableBackground();

        view.setText("视图");
        view.setFont(QFont("Nowar Warcraft Sans CN", 8));
        view.setFixedSize(45, 30);
        view.disableBackground();

        ros2.setText("ROS2");
        ros2.setFont(QFont("Nowar Warcraft Sans CN", 8));
        ros2.setFixedSize(45, 30);
        ros2.disableBackground();

        help.setText("帮助");
        help.setFont(QFont("Nowar Warcraft Sans CN", 8));
        help.setFixedSize(45, 30);
        help.disableBackground();

        name.setPlaceholderText("Point Cloud Shop");
        name.setFont(QFont("Nowar Warcraft Sans CN", 8));
        name.setFixedSize(200, 30);

        connect(&main, &RoundIconButton::clicked, []() {
            static auto themes = std::array {
                Theme::common::blue,
                Theme::common::green,
                Theme::common::grey,
                Theme::common::purple,
            };
            static auto index = 0;
            Theme::setTheme(themes[index++]);
            Theme::reloadTheme();

            if (index >= themes.size()) index = 0;
        });
    };

    RoundIconButton main;
    LineEdit name;
    PushButton file;
    PushButton edit;
    PushButton view;
    PushButton ros2;
    PushButton help;
};

TopArea::TopArea(QWidget* parent)
    : Rectangle(parent)
    , pimpl_(new Impl) {

    auto left = new QGridLayout;
    left->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    left->setMargin(5);
    left->setVerticalSpacing(5);
    left->setHorizontalSpacing(2);

    left->addLayout(pimpl_->main.horizontalWithSelf(), 0, 0, 1, 1);
    left->addWidget(&pimpl_->name, 0, 1, 1, 4);
    left->addWidget(&pimpl_->file, 1, 0, 1, 1);
    left->addWidget(&pimpl_->edit, 1, 1, 1, 1);
    left->addWidget(&pimpl_->view, 1, 2, 1, 1);
    left->addWidget(&pimpl_->ros2, 1, 3, 1, 1);
    left->addWidget(&pimpl_->help, 1, 4, 1, 1);

    auto right = new QHBoxLayout;
    right->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    right->setSpacing(10);
    right->setContentsMargins(5, 5, 5, 5);

    setLayout(left);
    setBackground(Theme::color("background"));
    setMaximumHeight(70);
}

TopArea::~TopArea() { delete pimpl_; }

void TopArea::setFileName(const QString& name) { pimpl_->name.setText(name); }
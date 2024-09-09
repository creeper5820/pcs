#include <QMouseEvent>
#include <QPushButton>

namespace workspace {
class OperateButton : public QPushButton {
public:
    explicit OperateButton(const QString& url)
        : QPushButton() {
        this->setIcon(QPixmap(url));
    }

protected:
    void mousePressEvent(QMouseEvent* event) override {
        if (Qt::RightButton == event->button())
            onRightMouseClicked();
        else
            onLeftMouseClicked();
    }

    virtual void onLeftMouseClicked() { }
    virtual void onRightMouseClicked() { }
};
}
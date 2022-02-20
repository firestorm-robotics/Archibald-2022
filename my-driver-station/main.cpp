#include <QtWidgets>
#include <QWidget>

class QPushButton;
class QTextBrowser;

// This is the declaration of our MainWidget class
// The definition/implementation is in mainwidget.cpp
class MainWidget : public QWidget
{
public:
    MainWidget(QWidget *parent = 0) : QWidget(parent){
       button_ = new QPushButton(tr("Push Me!"));
       textBrowser_ = new QTextBrowser();

       QGridLayout *mainLayout = new QGridLayout;
       mainLayout -> addWidget(button_,0,0);
       mainLayout -> addWidget(textBrowser_,1,0);
       setLayout(mainLayout);
       setWindowTitle(tr("Tyler's Drive Console"));
    }

private:
   QPushButton* button_;
   QTextBrowser* textBrowser_;
};

int main(int argc, char** argv){
    QApplication app(argc, argv);
    MainWidget w;
    w.show();
    return app.exec();
}

#include <QApplication>
#include <QMainWindow>
#include <QWebEngineView>
#include <QLineEdit>
#include <QPushButton>
#include <QToolBar>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QUrl>
#include <QIcon>
#include <QAction>
#include <QProgressBar>
#include <QLabel>

class BrowserWindow : public QMainWindow {
    Q_OBJECT

public:
    BrowserWindow(QWidget *parent = nullptr) : QMainWindow(parent) {
        // Create web view
        webView = new QWebEngineView(this);
        webView->setUrl(QUrl("https://www.google.com"));
        
        // Create address bar
        addressBar = new QLineEdit(this);
        addressBar->setPlaceholderText("Enter URL or search...");
        addressBar->setText("https://www.google.com");
        
        // Create navigation buttons
        backButton = new QPushButton("←", this);
        forwardButton = new QPushButton("→", this);
        refreshButton = new QPushButton("⟳", this);
        homeButton = new QPushButton("⌂", this);
        goButton = new QPushButton("Go", this);
        
        // Set button sizes
        backButton->setMaximumWidth(40);
        forwardButton->setMaximumWidth(40);
        refreshButton->setMaximumWidth(40);
        homeButton->setMaximumWidth(40);
        goButton->setMaximumWidth(60);
        
        // Create toolbar
        toolbar = new QToolBar(this);
        toolbar->addWidget(backButton);
        toolbar->addWidget(forwardButton);
        toolbar->addWidget(refreshButton);
        toolbar->addWidget(homeButton);
        toolbar->addWidget(addressBar);
        toolbar->addWidget(goButton);
        
        addToolBar(toolbar);
        
        // Create status bar
        statusBar = new QStatusBar(this);
        progressBar = new QProgressBar(this);
        progressBar->setVisible(false);
        progressBar->setMaximumWidth(200);
        statusLabel = new QLabel("Ready", this);
        statusBar->addWidget(statusLabel);
        statusBar->addPermanentWidget(progressBar);
        setStatusBar(statusBar);
        
        // Set central widget
        setCentralWidget(webView);
        
        // Connect signals and slots
        connect(goButton, &QPushButton::clicked, this, &BrowserWindow::navigateToUrl);
        connect(addressBar, &QLineEdit::returnPressed, this, &BrowserWindow::navigateToUrl);
        connect(backButton, &QPushButton::clicked, webView, &QWebEngineView::back);
        connect(forwardButton, &QPushButton::clicked, webView, &QWebEngineView::forward);
        connect(refreshButton, &QPushButton::clicked, webView, &QWebEngineView::reload);
        connect(homeButton, &QPushButton::clicked, this, &BrowserWindow::goHome);
        
        // Connect web view signals
        connect(webView, &QWebEngineView::urlChanged, this, &BrowserWindow::updateUrl);
        connect(webView, &QWebEngineView::titleChanged, this, &BrowserWindow::updateTitle);
        connect(webView, &QWebEngineView::loadProgress, this, &BrowserWindow::updateProgress);
        connect(webView, &QWebEngineView::loadFinished, this, &BrowserWindow::loadFinished);
        
        // Update button states
        connect(webView, &QWebEngineView::historyChanged, this, &BrowserWindow::updateNavigationButtons);
        
        // Set window properties
        setWindowTitle("Basic Browser");
        resize(1200, 800);
        
        updateNavigationButtons();
    }

private slots:
    void navigateToUrl() {
        QString url = addressBar->text();
        
        // Add https:// if no protocol is specified
        if (!url.startsWith("http://") && !url.startsWith("https://") && !url.startsWith("file://")) {
            // Check if it looks like a domain or IP
            if (url.contains('.') && !url.contains(' ')) {
                url = "https://" + url;
            } else {
                // Treat as search query
                url = "https://www.google.com/search?q=" + QUrl::toPercentEncoding(url);
            }
        }
        
        webView->setUrl(QUrl(url));
        statusLabel->setText("Loading...");
    }
    
    void updateUrl(const QUrl &url) {
        addressBar->setText(url.toString());
    }
    
    void updateTitle(const QString &title) {
        setWindowTitle(title + " - Basic Browser");
    }
    
    void updateProgress(int progress) {
        if (progress < 100) {
            progressBar->setValue(progress);
            progressBar->setVisible(true);
        } else {
            progressBar->setVisible(false);
        }
    }
    
    void loadFinished(bool success) {
        if (success) {
            statusLabel->setText("Page loaded successfully");
        } else {
            statusLabel->setText("Failed to load page");
        }
        progressBar->setVisible(false);
    }
    
    void goHome() {
        webView->setUrl(QUrl("https://www.google.com"));
    }
    
    void updateNavigationButtons() {
        backButton->setEnabled(webView->history()->canGoBack());
        forwardButton->setEnabled(webView->history()->canGoForward());
    }

private:
    QWebEngineView *webView;
    QLineEdit *addressBar;
    QPushButton *backButton;
    QPushButton *forwardButton;
    QPushButton *refreshButton;
    QPushButton *homeButton;
    QPushButton *goButton;
    QToolBar *toolbar;
    QStatusBar *statusBar;
    QProgressBar *progressBar;
    QLabel *statusLabel;
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    
    BrowserWindow window;
    window.show();
    
    return app.exec();
}

#include "browser.moc"


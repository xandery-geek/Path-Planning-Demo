#ifndef CUSTOM_EDIT_H
#define CUSTOM_EDIT_H

#include <QPlainTextEdit>
#include <QMenu>

class CustomPlainTextEdit : public QPlainTextEdit
{
    Q_OBJECT
public:
    explicit CustomPlainTextEdit(QWidget *parent = nullptr): QPlainTextEdit(parent) {};

protected:
    void contextMenuEvent(QContextMenuEvent* event) override {
        // Create the default context menu
        QMenu* menu = createStandardContextMenu();

        // Add a custom "Clear" action
        QAction* clearAction = menu->addAction("Clear Contents");

        // Connect the "Clear" action to the clear() method
        connect(clearAction, &QAction::triggered, this, &CustomPlainTextEdit::clear);

        // Show the context menu at the position of the event
        menu->exec(event->globalPos());

        // Clean up the menu to avoid memory leaks
        delete menu;
    }
};

#endif // CUSTOM_EDIT_H
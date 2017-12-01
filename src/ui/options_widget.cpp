//
// Created by tri on 26/09/2017.
//

#include "ui/options_widget.h"

namespace bkmap {

    OptionsWidget::OptionsWidget(QWidget* parent) : QWidget(parent) {
        setWindowFlags(Qt::Dialog);
        setWindowModality(Qt::ApplicationModal);

        QFont font;
        font.setPointSize(10);
        setFont(font);

        grid_layout_ = new QGridLayout(this);
        grid_layout_->setVerticalSpacing(3);
        grid_layout_->setAlignment(Qt::AlignTop);
        setLayout(grid_layout_);
    }

    void OptionsWidget::showEvent(QShowEvent* event) { ReadOptions(); }

    void OptionsWidget::closeEvent(QCloseEvent* event) { WriteOptions(); }

    void OptionsWidget::hideEvent(QHideEvent* event) { WriteOptions(); }

    void OptionsWidget::AddOptionRow(const std::string& label_text,
                                     QWidget* widget) {
        QLabel* label = new QLabel(tr(label_text.c_str()), this);
        label->setFont(font());
        label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid_layout_->addWidget(label, grid_layout_->rowCount(), 0);
        widget->setFont(font());
        grid_layout_->addWidget(widget, grid_layout_->rowCount() - 1, 1);
    }

    QSpinBox* OptionsWidget::AddOptionInt(int* option,
                                          const std::string& label_text,
                                          const int min, const int max, bool isShow) {
        QSpinBox* spinbox = new QSpinBox(this);
        spinbox->setMinimum(min);
        spinbox->setMaximum(max);
        if(isShow == true) {
            AddOptionRow(label_text, spinbox);
        }
        options_int_.emplace_back(spinbox, option);
        if(isShow == false){
            spinbox->hide();
        }
        return spinbox;
    }

    QDoubleSpinBox* OptionsWidget::AddOptionDouble(
            double* option, const std::string& label_text, const double min,
            const double max, const double step, const int decimals, bool isShow) {
        QDoubleSpinBox* spinbox = new QDoubleSpinBox(this);
        spinbox->setMinimum(min);
        spinbox->setMaximum(max);
        spinbox->setSingleStep(step);
        spinbox->setDecimals(decimals);
        if(isShow == true){
            AddOptionRow(label_text, spinbox);
        }

        options_double_.emplace_back(spinbox, option);
        if(isShow == false){
            spinbox->hide();
        }

        return spinbox;
    }

    QDoubleSpinBox* OptionsWidget::AddOptionDoubleLog(
            double* option, const std::string& label_text, const double min,
            const double max, const double step, const int decimals) {
        QDoubleSpinBox* spinbox = new QDoubleSpinBox(this);
        spinbox->setMinimum(min);
        spinbox->setMaximum(max);
        spinbox->setSingleStep(step);
        spinbox->setDecimals(decimals);

        AddOptionRow(label_text, spinbox);

        options_double_log_.emplace_back(spinbox, option);

        return spinbox;
    }

    QCheckBox* OptionsWidget::AddOptionBool(bool* option,
                                            const std::string& label_text, bool isShow) {
        QCheckBox* checkbox = new QCheckBox(this);
        if(isShow == true) {
            AddOptionRow(label_text, checkbox);
        }

        options_bool_.emplace_back(checkbox, option);
        if(isShow == false){
            checkbox->hide();
        }
        return checkbox;
    }

    QLineEdit* OptionsWidget::AddOptionText(std::string* option,
                                            const std::string& label_text, bool isShow) {
        QLineEdit* line_edit = new QLineEdit(this);
        if(isShow == true) {
            AddOptionRow(label_text, line_edit);
        }
        options_text_.emplace_back(line_edit, option);
        if(isShow == false){
            line_edit->hide();
        }
        return line_edit;
    }

    QLineEdit* OptionsWidget::AddOptionFilePath(std::string* option,
                                                const std::string& label_text, bool isShow) {
        QLineEdit* line_edit = new QLineEdit(this);
        if(isShow == true){
            AddOptionRow(label_text, line_edit);
        }

        auto SelectPathFunc = [this, line_edit]() {
            line_edit->setText(QFileDialog::getOpenFileName(this, tr("Select file")));
        };

        QPushButton* select_button = new QPushButton(tr("Select file"), this);
        select_button->setFont(font());
        connect(select_button, &QPushButton::released, this, SelectPathFunc);
        grid_layout_->addWidget(select_button, grid_layout_->rowCount(), 1);

        options_path_.emplace_back(line_edit, option);
        if(isShow == false){
            line_edit->hide();
            select_button->hide();
        }
        return line_edit;
    }

    QComboBox* OptionsWidget::AddDistanceOption(){

        QComboBox* cb = new QComboBox(this);
        cb->addItem("EUCLIDEAN");
        cb->addItem("MANHATTAN");
        cb->addItem("MINKOWSKI");
        cb->addItem("COSINE");
        AddOptionRow("Distance type", cb);
        return cb;
    }

    QLineEdit* OptionsWidget::AddOptionDirPath(std::string* option,
                                               const std::string& label_text) {
        QLineEdit* line_edit = new QLineEdit(this);

        AddOptionRow(label_text, line_edit);

        auto SelectPathFunc = [this, line_edit]() {
            line_edit->setText(
                    QFileDialog::getExistingDirectory(this, tr("Select folder")));
        };

        QPushButton* select_button = new QPushButton(tr("Select folder"), this);
        select_button->setFont(font());
        connect(select_button, &QPushButton::released, this, SelectPathFunc);
        grid_layout_->addWidget(select_button, grid_layout_->rowCount(), 1);

        options_path_.emplace_back(line_edit, option);

        return line_edit;
    }

    void OptionsWidget::AddSpacer() {
        QLabel* label = new QLabel("", this);
        label->setFont(font());
        grid_layout_->addWidget(label, grid_layout_->rowCount(), 0, 2, 1);
    }

    void OptionsWidget::AddSection(const std::string& label_text) {
        QLabel* label = new QLabel(tr(label_text.c_str()), this);
        label->setFont(font());
        label->setContentsMargins(0, 0, 0, 5);
        grid_layout_->addWidget(label, grid_layout_->rowCount(), 0, 1, 2,
                                Qt::AlignHCenter);
    }

    void OptionsWidget::ReadOptions() {
        for (auto& option : options_int_) {
            option.first->setValue(*option.second);
        }

        for (auto& option : options_double_) {
            option.first->setValue(*option.second);
        }

        for (auto& option : options_double_log_) {
            option.first->setValue(std::log10(*option.second));
        }

        for (auto& option : options_bool_) {
            option.first->setChecked(*option.second);
        }

        for (auto& option : options_text_) {
            option.first->setText(QString::fromStdString(*option.second));
        }

        for (auto& option : options_path_) {
            option.first->setText(QString::fromStdString(*option.second));
        }
    }

    void OptionsWidget::WriteOptions() {
        for (auto& option : options_int_) {
            *option.second = option.first->value();
        }

        for (auto& option : options_double_) {
            *option.second = option.first->value();
        }

        for (auto& option : options_double_log_) {
            *option.second = std::pow(10, option.first->value());
        }

        for (auto& option : options_bool_) {
            *option.second = option.first->isChecked();
        }

        for (auto& option : options_text_) {
            *option.second = option.first->text().toUtf8().constData();
        }

        for (auto& option : options_path_) {
            *option.second = option.first->text().toUtf8().constData();
        }
    }

}
//
// Created by tri on 26/09/2017.
//

#ifndef BKMAP_DATABASE_MANAGEMENT_WIDGET_H
#define BKMAP_DATABASE_MANAGEMENT_WIDGET_H

#include <unordered_map>

#include <QtCore>
#include <QtWidgets>

#include "base/database.h"
#include "ui/image_viewer_widget.h"
#include "util/misc.h"
#include "util/option_manager.h"

namespace bkmap {

////////////////////////////////////////////////////////////////////////////////
// Matches
////////////////////////////////////////////////////////////////////////////////

    class MatchesTab : public QWidget {
    public:
        MatchesTab() {}
        MatchesTab(QWidget* parent, OptionManager* options, Database* database);

        void Clear();

    protected:
        void InitializeTable(const QStringList& table_header);
        void ShowMatches();
        void FillTable();

        OptionManager* options_;
        Database* database_;

        const Image* image_;
        std::vector<std::pair<const Image*, FeatureMatches>> matches_;
        std::vector<int> configs_;
        std::vector<size_t> sorted_matches_idxs_;

        QTableWidget* table_widget_;
        QLabel* info_label_;
        FeatureImageViewerWidget* matches_viewer_widget_;
    };

    class RawMatchesTab : public MatchesTab {
    public:
        RawMatchesTab(QWidget* parent, OptionManager* options, Database* database);

        void Reload(const std::vector<Image>& images, const image_t image_id);
    };

    class InlierMatchesTab : public MatchesTab {
    public:
        InlierMatchesTab(QWidget* parent, OptionManager* options, Database* database);

        void Reload(const std::vector<Image>& images, const image_t image_id);
    };

    class MatchesWidget : public QWidget {
    public:
        MatchesWidget(QWidget* parent, OptionManager* options, Database* database);

        void ShowMatches(const std::vector<Image>& images, const image_t image_id);

    private:
        void closeEvent(QCloseEvent* event);

        QWidget* parent_;

        OptionManager* options_;

        QTabWidget* tab_widget_;
        RawMatchesTab* raw_matches_tab_;
        InlierMatchesTab* inlier_matches_tab_;
    };

////////////////////////////////////////////////////////////////////////////////
// Images, Cameras
////////////////////////////////////////////////////////////////////////////////

    class CameraTab : public QWidget {
    public:
        CameraTab(QWidget* parent, Database* database);

        void Reload();
        void Clear();

    private:
        void itemChanged(QTableWidgetItem* item);
        void Add();
        void SetModel();

        Database* database_;

        std::vector<Camera> cameras_;

        QTableWidget* table_widget_;
        QLabel* info_label_;
    };

    class ImageTab : public QWidget {
    public:
        ImageTab(QWidget* parent, CameraTab* camera_tab, OptionManager* options,
                 Database* database);

        void Reload();
        void Clear();

    private:
        void itemChanged(QTableWidgetItem* item);

        void ShowImage();
        void ShowMatches();
        void SetCamera();
        void SplitCamera();

        CameraTab* camera_tab_;

        OptionManager* options_;
        Database* database_;

        std::vector<Image> images_;

        QTableWidget* table_widget_;
        QLabel* info_label_;

        MatchesWidget* matches_widget_;

        FeatureImageViewerWidget* image_viewer_widget_;
    };

    class DatabaseManagementWidget : public QWidget {
    public:
        DatabaseManagementWidget(QWidget* parent, OptionManager* options);

    private:
        void showEvent(QShowEvent* event);
        void hideEvent(QHideEvent* event);

        void ClearMatches();
        void ClearInlierMatches();

        QWidget* parent_;

        OptionManager* options_;
        Database database_;

        QTabWidget* tab_widget_;
        ImageTab* image_tab_;
        CameraTab* camera_tab_;
    };

}

#endif //BKMAP_DATABASE_MANAGEMENT_WIDGET_H

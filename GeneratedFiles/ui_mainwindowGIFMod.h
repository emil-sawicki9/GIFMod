/********************************************************************************
** Form generated from reading UI file 'mainwindowGIFMod.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOWGIFMOD_H
#define UI_MAINWINDOWGIFMOD_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action_New;
    QAction *action_Open;
    QAction *action_Save;
    QAction *actionSave_As;
    QAction *actionE_xit;
    QAction *action_Undo;
    QAction *action_Redo;
    QAction *action_Cut;
    QAction *action_Copy;
    QAction *action_Paste;
    QAction *action_Select;
    QAction *actionSelect_All;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *action_Pan;
    QAction *actionContact_Us;
    QAction *actionAdd_Pond;
    QAction *actionAdd_Soil;
    QAction *actionAdd_Connector;
    QAction *actionZoom_All;
    QAction *actionlogWindow;
    QAction *actionAdd_Catchment_Area;
    QAction *actionAdd_Darcy_Block;
    QAction *actionAdd_Stora_ge;
    QAction *actionAdd_St_ream;
    QAction *actionDeterministic;
    QAction *actionBayesian;
    QAction *actionSoil;
    QAction *actionDarcy;
    QAction *actionStream;
    QAction *actionSoil_2;
    QAction *actionDarcy_2;
    QAction *actionCatchment_2;
    QAction *actionStream_2;
    QAction *actionSolverSettings;
    QAction *actionParticulate_Phase;
    QAction *actionConstituents;
    QAction *actionReactions;
    QAction *actionRun_Model;
    QAction *actionLoad_Reaction_Network;
    QAction *actionSave_Reaction_Network;
    QAction *actionClear_Reaction_Network;
    QAction *actionRun_Model_from_Script;
    QAction *action_Hydraulic_Outputs;
    QAction *actionExport_to_Script_Language;
    QAction *actionAbout;
    QAction *actionRun_Inverse_Model;
    QAction *actionNewExperiment;
    QAction *actionCopyFromCurrentExperiment;
    QAction *actionProjectSettings;
    QAction *actionClimateSettings;
    QAction *actionremoveCurrentExperiment;
    QAction *actionShowRuntimeWindow;
    QAction *actionConnectors;
    QAction *actionStorage;
    QAction *actionHead;
    QAction *actionMoistureContent;
    QAction *actionWaterDepth;
    QAction *actionEvaporationRate;
    QAction *actioncolorCodeHead;
    QAction *actioncolorCodeStorage;
    QAction *actioncolorCodeWaterDepth;
    QAction *actioncolorCodeMoistureContent;
    QAction *actioncolorCodeEvaporationRate;
    QAction *action123;
    QAction *actionAbout2;
    QAction *actionRecent;
    QAction *actionColorCodeConnectorFlow;
    QAction *actionColorCodeConnectorVelocity;
    QAction *actionColorCodeConnectorArea;
    QAction *actionColorCodeConnectorVaporExchangeEate;
    QAction *actionReset_colors;
    QAction *actionNew_from_template;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QMenu *menuRecent;
    QMenu *menu_Help;
    QMenu *menu_Edit;
    QMenu *menu_Window;
    QMenu *menu_Model;
    QMenu *menuAdd_Block;
    QMenu *menuPreferences;
    QMenu *menuPost_Processing;
    QMenu *menuBlocks;
    QMenu *menuConnectors;
    QMenu *menuWaterQuality;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;
    QToolBar *toolBar_4;
    QToolBar *experimentsToolbar;
    QToolBar *toolBar_3;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setWindowModality(Qt::NonModal);
        MainWindow->resize(1177, 791);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMouseTracking(true);
        QIcon icon;
        icon.addFile(QStringLiteral("Icons/GIFMod.ico"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        MainWindow->setWindowOpacity(1);
        MainWindow->setToolTipDuration(1);
        MainWindow->setIconSize(QSize(30, 30));
        MainWindow->setDocumentMode(false);
        MainWindow->setTabShape(QTabWidget::Rounded);
        MainWindow->setUnifiedTitleAndToolBarOnMac(false);
        action_New = new QAction(MainWindow);
        action_New->setObjectName(QStringLiteral("action_New"));
        QIcon icon1;
        icon1.addFile(QStringLiteral("Icons/Document-Blank-icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_New->setIcon(icon1);
        action_Open = new QAction(MainWindow);
        action_Open->setObjectName(QStringLiteral("action_Open"));
        QIcon icon2;
        icon2.addFile(QStringLiteral("Icons/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QStringLiteral("Icons/open.ico"), QSize(), QIcon::Normal, QIcon::On);
        action_Open->setIcon(icon2);
        action_Save = new QAction(MainWindow);
        action_Save->setObjectName(QStringLiteral("action_Save"));
        QIcon icon3;
        icon3.addFile(QStringLiteral("Icons/Save.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Save->setIcon(icon3);
        actionSave_As = new QAction(MainWindow);
        actionSave_As->setObjectName(QStringLiteral("actionSave_As"));
        actionE_xit = new QAction(MainWindow);
        actionE_xit->setObjectName(QStringLiteral("actionE_xit"));
        action_Undo = new QAction(MainWindow);
        action_Undo->setObjectName(QStringLiteral("action_Undo"));
        QIcon icon4;
        icon4.addFile(QStringLiteral("Icons/undo.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Undo->setIcon(icon4);
        action_Redo = new QAction(MainWindow);
        action_Redo->setObjectName(QStringLiteral("action_Redo"));
        QIcon icon5;
        icon5.addFile(QStringLiteral("Icons/1439255778_Redo.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Redo->setIcon(icon5);
        action_Cut = new QAction(MainWindow);
        action_Cut->setObjectName(QStringLiteral("action_Cut"));
        QIcon icon6;
        icon6.addFile(QStringLiteral("Icons/cut-icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Cut->setIcon(icon6);
        action_Copy = new QAction(MainWindow);
        action_Copy->setObjectName(QStringLiteral("action_Copy"));
        QIcon icon7;
        icon7.addFile(QStringLiteral("Icons/copy.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Copy->setIcon(icon7);
        action_Paste = new QAction(MainWindow);
        action_Paste->setObjectName(QStringLiteral("action_Paste"));
        QIcon icon8;
        icon8.addFile(QStringLiteral("Icons/Paste.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Paste->setIcon(icon8);
        action_Select = new QAction(MainWindow);
        action_Select->setObjectName(QStringLiteral("action_Select"));
        action_Select->setCheckable(true);
        action_Select->setChecked(true);
        QIcon icon9;
        icon9.addFile(QStringLiteral("Icons/arrow.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Select->setIcon(icon9);
        actionSelect_All = new QAction(MainWindow);
        actionSelect_All->setObjectName(QStringLiteral("actionSelect_All"));
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName(QStringLiteral("actionZoom_In"));
        QIcon icon10;
        icon10.addFile(QStringLiteral("Icons/zoom_in.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_In->setIcon(icon10);
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName(QStringLiteral("actionZoom_Out"));
        QIcon icon11;
        icon11.addFile(QStringLiteral("Icons/zoom_out.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_Out->setIcon(icon11);
        action_Pan = new QAction(MainWindow);
        action_Pan->setObjectName(QStringLiteral("action_Pan"));
        action_Pan->setCheckable(true);
        QIcon icon12;
        icon12.addFile(QStringLiteral("Icons/pan-icon.gif"), QSize(), QIcon::Normal, QIcon::Off);
        action_Pan->setIcon(icon12);
        actionContact_Us = new QAction(MainWindow);
        actionContact_Us->setObjectName(QStringLiteral("actionContact_Us"));
        actionAdd_Pond = new QAction(MainWindow);
        actionAdd_Pond->setObjectName(QStringLiteral("actionAdd_Pond"));
        QIcon icon13;
        icon13.addFile(QStringLiteral("Icons/Pond.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Pond->setIcon(icon13);
        actionAdd_Soil = new QAction(MainWindow);
        actionAdd_Soil->setObjectName(QStringLiteral("actionAdd_Soil"));
        QIcon icon14;
        icon14.addFile(QStringLiteral("Icons/isometric_sand_block_icon__hd__by_memoryleakxxx-d8e3mwa.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Soil->setIcon(icon14);
        actionAdd_Connector = new QAction(MainWindow);
        actionAdd_Connector->setObjectName(QStringLiteral("actionAdd_Connector"));
        actionAdd_Connector->setCheckable(true);
        QIcon icon15;
        icon15.addFile(QStringLiteral("Icons/draw-connector.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Connector->setIcon(icon15);
        actionZoom_All = new QAction(MainWindow);
        actionZoom_All->setObjectName(QStringLiteral("actionZoom_All"));
        QIcon icon16;
        icon16.addFile(QStringLiteral("Icons/Full_screen_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionZoom_All->setIcon(icon16);
        actionlogWindow = new QAction(MainWindow);
        actionlogWindow->setObjectName(QStringLiteral("actionlogWindow"));
        actionlogWindow->setCheckable(true);
        QIcon icon17;
        icon17.addFile(QStringLiteral("Icons/log_icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionlogWindow->setIcon(icon17);
        actionAdd_Catchment_Area = new QAction(MainWindow);
        actionAdd_Catchment_Area->setObjectName(QStringLiteral("actionAdd_Catchment_Area"));
        QIcon icon18;
        icon18.addFile(QStringLiteral("Icons/catchment.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Catchment_Area->setIcon(icon18);
        actionAdd_Darcy_Block = new QAction(MainWindow);
        actionAdd_Darcy_Block->setObjectName(QStringLiteral("actionAdd_Darcy_Block"));
        QIcon icon19;
        icon19.addFile(QStringLiteral("Icons/Darcy.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Darcy_Block->setIcon(icon19);
        actionAdd_Stora_ge = new QAction(MainWindow);
        actionAdd_Stora_ge->setObjectName(QStringLiteral("actionAdd_Stora_ge"));
        QIcon icon20;
        icon20.addFile(QStringLiteral("Icons/Storage.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_Stora_ge->setIcon(icon20);
        actionAdd_St_ream = new QAction(MainWindow);
        actionAdd_St_ream->setObjectName(QStringLiteral("actionAdd_St_ream"));
        QIcon icon21;
        icon21.addFile(QStringLiteral("Icons/Stream.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionAdd_St_ream->setIcon(icon21);
        actionDeterministic = new QAction(MainWindow);
        actionDeterministic->setObjectName(QStringLiteral("actionDeterministic"));
        actionBayesian = new QAction(MainWindow);
        actionBayesian->setObjectName(QStringLiteral("actionBayesian"));
        actionSoil = new QAction(MainWindow);
        actionSoil->setObjectName(QStringLiteral("actionSoil"));
        actionDarcy = new QAction(MainWindow);
        actionDarcy->setObjectName(QStringLiteral("actionDarcy"));
        actionStream = new QAction(MainWindow);
        actionStream->setObjectName(QStringLiteral("actionStream"));
        actionSoil_2 = new QAction(MainWindow);
        actionSoil_2->setObjectName(QStringLiteral("actionSoil_2"));
        actionDarcy_2 = new QAction(MainWindow);
        actionDarcy_2->setObjectName(QStringLiteral("actionDarcy_2"));
        actionCatchment_2 = new QAction(MainWindow);
        actionCatchment_2->setObjectName(QStringLiteral("actionCatchment_2"));
        actionStream_2 = new QAction(MainWindow);
        actionStream_2->setObjectName(QStringLiteral("actionStream_2"));
        actionSolverSettings = new QAction(MainWindow);
        actionSolverSettings->setObjectName(QStringLiteral("actionSolverSettings"));
        actionParticulate_Phase = new QAction(MainWindow);
        actionParticulate_Phase->setObjectName(QStringLiteral("actionParticulate_Phase"));
        actionConstituents = new QAction(MainWindow);
        actionConstituents->setObjectName(QStringLiteral("actionConstituents"));
        actionReactions = new QAction(MainWindow);
        actionReactions->setObjectName(QStringLiteral("actionReactions"));
        actionRun_Model = new QAction(MainWindow);
        actionRun_Model->setObjectName(QStringLiteral("actionRun_Model"));
        QIcon icon22;
        icon22.addFile(QStringLiteral("Icons/gear_run.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon22.addFile(QStringLiteral("Icons/gear_run.png"), QSize(), QIcon::Normal, QIcon::On);
        actionRun_Model->setIcon(icon22);
        actionLoad_Reaction_Network = new QAction(MainWindow);
        actionLoad_Reaction_Network->setObjectName(QStringLiteral("actionLoad_Reaction_Network"));
        actionSave_Reaction_Network = new QAction(MainWindow);
        actionSave_Reaction_Network->setObjectName(QStringLiteral("actionSave_Reaction_Network"));
        actionClear_Reaction_Network = new QAction(MainWindow);
        actionClear_Reaction_Network->setObjectName(QStringLiteral("actionClear_Reaction_Network"));
        actionRun_Model_from_Script = new QAction(MainWindow);
        actionRun_Model_from_Script->setObjectName(QStringLiteral("actionRun_Model_from_Script"));
        QIcon icon23;
        icon23.addFile(QStringLiteral("Icons/runScript.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRun_Model_from_Script->setIcon(icon23);
        action_Hydraulic_Outputs = new QAction(MainWindow);
        action_Hydraulic_Outputs->setObjectName(QStringLiteral("action_Hydraulic_Outputs"));
        actionExport_to_Script_Language = new QAction(MainWindow);
        actionExport_to_Script_Language->setObjectName(QStringLiteral("actionExport_to_Script_Language"));
        QIcon icon24;
        icon24.addFile(QStringLiteral("Icons/generate script.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionExport_to_Script_Language->setIcon(icon24);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionRun_Inverse_Model = new QAction(MainWindow);
        actionRun_Inverse_Model->setObjectName(QStringLiteral("actionRun_Inverse_Model"));
        QIcon icon25;
        icon25.addFile(QStringLiteral("Icons/backwards-running.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionRun_Inverse_Model->setIcon(icon25);
        actionNewExperiment = new QAction(MainWindow);
        actionNewExperiment->setObjectName(QStringLiteral("actionNewExperiment"));
        QIcon icon26;
        icon26.addFile(QStringLiteral("Icons/experiment2.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNewExperiment->setIcon(icon26);
        actionCopyFromCurrentExperiment = new QAction(MainWindow);
        actionCopyFromCurrentExperiment->setObjectName(QStringLiteral("actionCopyFromCurrentExperiment"));
        QIcon icon27;
        icon27.addFile(QStringLiteral("Icons/experiment1.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionCopyFromCurrentExperiment->setIcon(icon27);
        actionProjectSettings = new QAction(MainWindow);
        actionProjectSettings->setObjectName(QStringLiteral("actionProjectSettings"));
        actionClimateSettings = new QAction(MainWindow);
        actionClimateSettings->setObjectName(QStringLiteral("actionClimateSettings"));
        actionremoveCurrentExperiment = new QAction(MainWindow);
        actionremoveCurrentExperiment->setObjectName(QStringLiteral("actionremoveCurrentExperiment"));
        QIcon icon28;
        icon28.addFile(QStringLiteral("Icons/delete-big.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionremoveCurrentExperiment->setIcon(icon28);
        actionShowRuntimeWindow = new QAction(MainWindow);
        actionShowRuntimeWindow->setObjectName(QStringLiteral("actionShowRuntimeWindow"));
        QIcon icon29;
        icon29.addFile(QStringLiteral("Icons/oscilloscope.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionShowRuntimeWindow->setIcon(icon29);
        actionConnectors = new QAction(MainWindow);
        actionConnectors->setObjectName(QStringLiteral("actionConnectors"));
        actionStorage = new QAction(MainWindow);
        actionStorage->setObjectName(QStringLiteral("actionStorage"));
        actionHead = new QAction(MainWindow);
        actionHead->setObjectName(QStringLiteral("actionHead"));
        actionMoistureContent = new QAction(MainWindow);
        actionMoistureContent->setObjectName(QStringLiteral("actionMoistureContent"));
        actionWaterDepth = new QAction(MainWindow);
        actionWaterDepth->setObjectName(QStringLiteral("actionWaterDepth"));
        actionEvaporationRate = new QAction(MainWindow);
        actionEvaporationRate->setObjectName(QStringLiteral("actionEvaporationRate"));
        actioncolorCodeHead = new QAction(MainWindow);
        actioncolorCodeHead->setObjectName(QStringLiteral("actioncolorCodeHead"));
        actioncolorCodeStorage = new QAction(MainWindow);
        actioncolorCodeStorage->setObjectName(QStringLiteral("actioncolorCodeStorage"));
        actioncolorCodeWaterDepth = new QAction(MainWindow);
        actioncolorCodeWaterDepth->setObjectName(QStringLiteral("actioncolorCodeWaterDepth"));
        actioncolorCodeMoistureContent = new QAction(MainWindow);
        actioncolorCodeMoistureContent->setObjectName(QStringLiteral("actioncolorCodeMoistureContent"));
        actioncolorCodeEvaporationRate = new QAction(MainWindow);
        actioncolorCodeEvaporationRate->setObjectName(QStringLiteral("actioncolorCodeEvaporationRate"));
        action123 = new QAction(MainWindow);
        action123->setObjectName(QStringLiteral("action123"));
        actionAbout2 = new QAction(MainWindow);
        actionAbout2->setObjectName(QStringLiteral("actionAbout2"));
        actionRecent = new QAction(MainWindow);
        actionRecent->setObjectName(QStringLiteral("actionRecent"));
        actionColorCodeConnectorFlow = new QAction(MainWindow);
        actionColorCodeConnectorFlow->setObjectName(QStringLiteral("actionColorCodeConnectorFlow"));
        actionColorCodeConnectorVelocity = new QAction(MainWindow);
        actionColorCodeConnectorVelocity->setObjectName(QStringLiteral("actionColorCodeConnectorVelocity"));
        actionColorCodeConnectorArea = new QAction(MainWindow);
        actionColorCodeConnectorArea->setObjectName(QStringLiteral("actionColorCodeConnectorArea"));
        actionColorCodeConnectorVaporExchangeEate = new QAction(MainWindow);
        actionColorCodeConnectorVaporExchangeEate->setObjectName(QStringLiteral("actionColorCodeConnectorVaporExchangeEate"));
        actionReset_colors = new QAction(MainWindow);
        actionReset_colors->setObjectName(QStringLiteral("actionReset_colors"));
        actionNew_from_template = new QAction(MainWindow);
        actionNew_from_template->setObjectName(QStringLiteral("actionNew_from_template"));
        QIcon icon30;
        icon30.addFile(QStringLiteral("../Icons/wizard blue.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionNew_from_template->setIcon(icon30);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1177, 21));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        menuRecent = new QMenu(menu_File);
        menuRecent->setObjectName(QStringLiteral("menuRecent"));
        menu_Help = new QMenu(menuBar);
        menu_Help->setObjectName(QStringLiteral("menu_Help"));
        menu_Edit = new QMenu(menuBar);
        menu_Edit->setObjectName(QStringLiteral("menu_Edit"));
        menu_Window = new QMenu(menuBar);
        menu_Window->setObjectName(QStringLiteral("menu_Window"));
        menu_Model = new QMenu(menuBar);
        menu_Model->setObjectName(QStringLiteral("menu_Model"));
        menuAdd_Block = new QMenu(menu_Model);
        menuAdd_Block->setObjectName(QStringLiteral("menuAdd_Block"));
        menuPreferences = new QMenu(menuBar);
        menuPreferences->setObjectName(QStringLiteral("menuPreferences"));
        menuPost_Processing = new QMenu(menuBar);
        menuPost_Processing->setObjectName(QStringLiteral("menuPost_Processing"));
        menuBlocks = new QMenu(menuPost_Processing);
        menuBlocks->setObjectName(QStringLiteral("menuBlocks"));
        menuConnectors = new QMenu(menuPost_Processing);
        menuConnectors->setObjectName(QStringLiteral("menuConnectors"));
        menuWaterQuality = new QMenu(menuPost_Processing);
        menuWaterQuality->setObjectName(QStringLiteral("menuWaterQuality"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        mainToolBar->setMouseTracking(true);
        mainToolBar->setWindowOpacity(1);
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        toolBar_2 = new QToolBar(MainWindow);
        toolBar_2->setObjectName(QStringLiteral("toolBar_2"));
        MainWindow->addToolBar(Qt::LeftToolBarArea, toolBar_2);
        toolBar_4 = new QToolBar(MainWindow);
        toolBar_4->setObjectName(QStringLiteral("toolBar_4"));
        toolBar_4->setEnabled(true);
        MainWindow->addToolBar(Qt::LeftToolBarArea, toolBar_4);
        experimentsToolbar = new QToolBar(MainWindow);
        experimentsToolbar->setObjectName(QStringLiteral("experimentsToolbar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, experimentsToolbar);
        toolBar_3 = new QToolBar(MainWindow);
        toolBar_3->setObjectName(QStringLiteral("toolBar_3"));
        MainWindow->addToolBar(Qt::LeftToolBarArea, toolBar_3);

        menuBar->addAction(menu_File->menuAction());
        menuBar->addAction(menu_Edit->menuAction());
        menuBar->addAction(menu_Window->menuAction());
        menuBar->addAction(menu_Model->menuAction());
        menuBar->addAction(menuPreferences->menuAction());
        menuBar->addAction(menuPost_Processing->menuAction());
        menuBar->addAction(menu_Help->menuAction());
        menu_File->addAction(action_New);
        menu_File->addAction(actionNew_from_template);
        menu_File->addAction(action_Open);
        menu_File->addAction(action_Save);
        menu_File->addAction(actionSave_As);
        menu_File->addSeparator();
        menu_File->addAction(menuRecent->menuAction());
        menu_File->addSeparator();
        menu_File->addAction(actionClear_Reaction_Network);
        menu_File->addAction(actionLoad_Reaction_Network);
        menu_File->addAction(actionSave_Reaction_Network);
        menu_File->addSeparator();
        menu_File->addAction(actionE_xit);
        menu_Help->addAction(actionContact_Us);
        menu_Help->addAction(actionAbout);
        menu_Edit->addAction(action_Undo);
        menu_Edit->addAction(action_Redo);
        menu_Edit->addSeparator();
        menu_Edit->addAction(actionSelect_All);
        menu_Edit->addSeparator();
        menu_Edit->addAction(action_Cut);
        menu_Edit->addAction(action_Copy);
        menu_Edit->addAction(action_Paste);
        menu_Window->addAction(actionZoom_In);
        menu_Window->addAction(actionZoom_Out);
        menu_Window->addAction(actionZoom_All);
        menu_Window->addSeparator();
        menu_Window->addAction(action_Select);
        menu_Window->addAction(action_Pan);
        menu_Window->addSeparator();
        menu_Window->addAction(actionlogWindow);
        menu_Window->addAction(actionShowRuntimeWindow);
        menu_Model->addAction(menuAdd_Block->menuAction());
        menu_Model->addAction(actionReactions);
        menu_Model->addSeparator();
        menu_Model->addAction(actionRun_Model);
        menu_Model->addAction(actionExport_to_Script_Language);
        menuAdd_Block->addAction(actionAdd_Pond);
        menuAdd_Block->addAction(actionAdd_Soil);
        menuAdd_Block->addAction(actionAdd_Catchment_Area);
        menuAdd_Block->addAction(actionAdd_Darcy_Block);
        menuAdd_Block->addAction(actionAdd_Stora_ge);
        menuAdd_Block->addAction(actionAdd_St_ream);
        menuPreferences->addAction(actionProjectSettings);
        menuPreferences->addAction(actionClimateSettings);
        menuPreferences->addAction(actionSolverSettings);
        menuPost_Processing->addAction(menuBlocks->menuAction());
        menuPost_Processing->addAction(menuConnectors->menuAction());
        menuPost_Processing->addAction(menuWaterQuality->menuAction());
        menuPost_Processing->addAction(actionReset_colors);
        menuBlocks->addAction(actioncolorCodeHead);
        menuBlocks->addAction(actioncolorCodeStorage);
        menuBlocks->addAction(actioncolorCodeWaterDepth);
        menuBlocks->addAction(actioncolorCodeMoistureContent);
        menuBlocks->addAction(actioncolorCodeEvaporationRate);
        menuConnectors->addAction(actionColorCodeConnectorFlow);
        menuConnectors->addAction(actionColorCodeConnectorVelocity);
        menuConnectors->addAction(actionColorCodeConnectorArea);
        menuConnectors->addAction(actionColorCodeConnectorVaporExchangeEate);
        menuWaterQuality->addSeparator();
        mainToolBar->addAction(action_New);
        mainToolBar->addAction(actionNew_from_template);
        mainToolBar->addAction(action_Open);
        mainToolBar->addAction(action_Save);
        mainToolBar->addSeparator();
        mainToolBar->addAction(action_Undo);
        mainToolBar->addAction(action_Redo);
        mainToolBar->addSeparator();
        mainToolBar->addAction(action_Cut);
        mainToolBar->addAction(action_Copy);
        mainToolBar->addAction(action_Paste);
        toolBar->addAction(actionAdd_Pond);
        toolBar->addSeparator();
        toolBar->addAction(actionAdd_Soil);
        toolBar->addAction(actionAdd_Darcy_Block);
        toolBar->addAction(actionAdd_Stora_ge);
        toolBar->addSeparator();
        toolBar->addAction(actionAdd_Catchment_Area);
        toolBar->addAction(actionAdd_St_ream);
        toolBar->addSeparator();
        toolBar_2->addAction(actionZoom_In);
        toolBar_2->addAction(actionZoom_Out);
        toolBar_2->addAction(actionZoom_All);
        toolBar_4->addAction(actionRun_Model);
        toolBar_4->addAction(actionRun_Inverse_Model);
        toolBar_4->addAction(actionlogWindow);
        toolBar_4->addAction(actionShowRuntimeWindow);
        toolBar_4->addSeparator();
        toolBar_4->addAction(actionExport_to_Script_Language);
        toolBar_4->addAction(actionRun_Model_from_Script);
        experimentsToolbar->addAction(actionNewExperiment);
        experimentsToolbar->addAction(actionCopyFromCurrentExperiment);
        experimentsToolbar->addAction(actionremoveCurrentExperiment);
        experimentsToolbar->addSeparator();

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Green Infrastructure Flexible Model (GIFMod)", 0));
        action_New->setText(QApplication::translate("MainWindow", "&New", 0));
        action_Open->setText(QApplication::translate("MainWindow", "&Open", 0));
        action_Open->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", 0));
        action_Save->setText(QApplication::translate("MainWindow", "&Save", 0));
        action_Save->setShortcut(QApplication::translate("MainWindow", "Ctrl+S", 0));
        actionSave_As->setText(QApplication::translate("MainWindow", "Save &As", 0));
        actionE_xit->setText(QApplication::translate("MainWindow", "E&xit", 0));
        action_Undo->setText(QApplication::translate("MainWindow", "&Undo", 0));
        action_Undo->setShortcut(QApplication::translate("MainWindow", "Ctrl+Z", 0));
        action_Redo->setText(QApplication::translate("MainWindow", "&Redo", 0));
        action_Cut->setText(QApplication::translate("MainWindow", "C&ut", 0));
        action_Copy->setText(QApplication::translate("MainWindow", "&Copy", 0));
        action_Copy->setShortcut(QApplication::translate("MainWindow", "Ctrl+C", 0));
        action_Paste->setText(QApplication::translate("MainWindow", "&Paste", 0));
        action_Paste->setShortcut(QApplication::translate("MainWindow", "Ctrl+V", 0));
        action_Select->setText(QApplication::translate("MainWindow", "&Select", 0));
        actionSelect_All->setText(QApplication::translate("MainWindow", "Select &All", 0));
        actionZoom_In->setText(QApplication::translate("MainWindow", "Zoom &in", 0));
        actionZoom_Out->setText(QApplication::translate("MainWindow", "Zoom &out", 0));
        action_Pan->setText(QApplication::translate("MainWindow", "&Pan", 0));
        actionContact_Us->setText(QApplication::translate("MainWindow", "Contact &Us", 0));
        actionAdd_Pond->setText(QApplication::translate("MainWindow", "Add &Pond", 0));
        actionAdd_Soil->setText(QApplication::translate("MainWindow", "Add &Soil", 0));
        actionAdd_Connector->setText(QApplication::translate("MainWindow", "Interface", 0));
        actionZoom_All->setText(QApplication::translate("MainWindow", "Zoom &all", 0));
        actionlogWindow->setText(QApplication::translate("MainWindow", "Show &log window", 0));
        actionlogWindow->setShortcut(QApplication::translate("MainWindow", "Ctrl+L", 0));
        actionAdd_Catchment_Area->setText(QApplication::translate("MainWindow", "Add Catchment", 0));
        actionAdd_Darcy_Block->setText(QApplication::translate("MainWindow", "Add &Darcy Block", 0));
        actionAdd_Stora_ge->setText(QApplication::translate("MainWindow", "Add Stora&ge", 0));
        actionAdd_St_ream->setText(QApplication::translate("MainWindow", "Add St&ream", 0));
        actionDeterministic->setText(QApplication::translate("MainWindow", "Deterministic", 0));
        actionBayesian->setText(QApplication::translate("MainWindow", "Bayesian", 0));
        actionSoil->setText(QApplication::translate("MainWindow", "Soil", 0));
        actionDarcy->setText(QApplication::translate("MainWindow", "Darcy", 0));
        actionStream->setText(QApplication::translate("MainWindow", "Stream", 0));
        actionSoil_2->setText(QApplication::translate("MainWindow", "Soil", 0));
        actionDarcy_2->setText(QApplication::translate("MainWindow", "Darcy", 0));
        actionCatchment_2->setText(QApplication::translate("MainWindow", "Catchment", 0));
        actionStream_2->setText(QApplication::translate("MainWindow", "Stream", 0));
        actionSolverSettings->setText(QApplication::translate("MainWindow", "&Solver settings", 0));
        actionParticulate_Phase->setText(QApplication::translate("MainWindow", "Particulate Phase", 0));
        actionConstituents->setText(QApplication::translate("MainWindow", "Constituents", 0));
        actionReactions->setText(QApplication::translate("MainWindow", "Reactions", 0));
        actionRun_Model->setText(QApplication::translate("MainWindow", "Run Model", 0));
        actionRun_Model->setShortcut(QApplication::translate("MainWindow", "F5", 0));
        actionLoad_Reaction_Network->setText(QApplication::translate("MainWindow", "Load Reaction Network", 0));
        actionSave_Reaction_Network->setText(QApplication::translate("MainWindow", "Save Reaction Network", 0));
#ifndef QT_NO_TOOLTIP
        actionSave_Reaction_Network->setToolTip(QApplication::translate("MainWindow", "Save Reaction Network", 0));
#endif // QT_NO_TOOLTIP
        actionClear_Reaction_Network->setText(QApplication::translate("MainWindow", "Clear Reaction Network", 0));
        actionRun_Model_from_Script->setText(QApplication::translate("MainWindow", "Run Model from Script", 0));
        action_Hydraulic_Outputs->setText(QApplication::translate("MainWindow", "&Hydraulic Outputs", 0));
        actionExport_to_Script_Language->setText(QApplication::translate("MainWindow", "Export to &Script Language", 0));
        actionAbout->setText(QApplication::translate("MainWindow", "About GIFMod", 0));
        actionRun_Inverse_Model->setText(QApplication::translate("MainWindow", "Run Inverse Model", 0));
        actionNewExperiment->setText(QApplication::translate("MainWindow", "New experiment", 0));
#ifndef QT_NO_TOOLTIP
        actionNewExperiment->setToolTip(QApplication::translate("MainWindow", "New experiment", 0));
#endif // QT_NO_TOOLTIP
        actionCopyFromCurrentExperiment->setText(QApplication::translate("MainWindow", "Copy from current experiment", 0));
        actionProjectSettings->setText(QApplication::translate("MainWindow", "&Project settings", 0));
        actionClimateSettings->setText(QApplication::translate("MainWindow", "&Climate settings", 0));
        actionremoveCurrentExperiment->setText(QApplication::translate("MainWindow", "Remove current experiment", 0));
#ifndef QT_NO_TOOLTIP
        actionremoveCurrentExperiment->setToolTip(QApplication::translate("MainWindow", "Remove Current Experiment", 0));
#endif // QT_NO_TOOLTIP
        actionShowRuntimeWindow->setText(QApplication::translate("MainWindow", "Show &runtime window", 0));
        actionConnectors->setText(QApplication::translate("MainWindow", "Connectors", 0));
        actionStorage->setText(QApplication::translate("MainWindow", "Storage", 0));
        actionHead->setText(QApplication::translate("MainWindow", "Head", 0));
        actionMoistureContent->setText(QApplication::translate("MainWindow", "Moisture content", 0));
        actionWaterDepth->setText(QApplication::translate("MainWindow", "Water depth", 0));
        actionEvaporationRate->setText(QApplication::translate("MainWindow", "Evaporation rate", 0));
        actioncolorCodeHead->setText(QApplication::translate("MainWindow", "Head", 0));
        actioncolorCodeStorage->setText(QApplication::translate("MainWindow", "Storage", 0));
        actioncolorCodeWaterDepth->setText(QApplication::translate("MainWindow", "Water depth", 0));
        actioncolorCodeMoistureContent->setText(QApplication::translate("MainWindow", "Moisture content", 0));
        actioncolorCodeEvaporationRate->setText(QApplication::translate("MainWindow", "Evaporation rate", 0));
        action123->setText(QApplication::translate("MainWindow", "123", 0));
        actionAbout2->setText(QApplication::translate("MainWindow", "About", 0));
        actionRecent->setText(QApplication::translate("MainWindow", "Recent", 0));
        actionColorCodeConnectorFlow->setText(QApplication::translate("MainWindow", "Flow", 0));
        actionColorCodeConnectorVelocity->setText(QApplication::translate("MainWindow", "Velocity", 0));
        actionColorCodeConnectorArea->setText(QApplication::translate("MainWindow", "Area", 0));
        actionColorCodeConnectorVaporExchangeEate->setText(QApplication::translate("MainWindow", "Vapor exchange rate", 0));
        actionReset_colors->setText(QApplication::translate("MainWindow", "Reset colors", 0));
        actionNew_from_template->setText(QApplication::translate("MainWindow", "New from &template...", 0));
        menu_File->setTitle(QApplication::translate("MainWindow", "&File", 0));
        menuRecent->setTitle(QApplication::translate("MainWindow", "Recent", 0));
        menu_Help->setTitle(QApplication::translate("MainWindow", "&Help", 0));
        menu_Edit->setTitle(QApplication::translate("MainWindow", "&Edit", 0));
        menu_Window->setTitle(QApplication::translate("MainWindow", "&View", 0));
        menu_Model->setTitle(QApplication::translate("MainWindow", "&Model", 0));
        menuAdd_Block->setTitle(QApplication::translate("MainWindow", "Block", 0));
        menuPreferences->setTitle(QApplication::translate("MainWindow", "Preferences", 0));
        menuPost_Processing->setTitle(QApplication::translate("MainWindow", "Post Processing", 0));
        menuBlocks->setTitle(QApplication::translate("MainWindow", "Blocks", 0));
        menuConnectors->setTitle(QApplication::translate("MainWindow", "Connectors", 0));
        menuWaterQuality->setTitle(QApplication::translate("MainWindow", "Water quality", 0));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0));
        toolBar_2->setWindowTitle(QApplication::translate("MainWindow", "toolBar_2", 0));
        toolBar_4->setWindowTitle(QApplication::translate("MainWindow", "toolBar_4", 0));
        experimentsToolbar->setWindowTitle(QApplication::translate("MainWindow", "experimentsToolbar", 0));
        toolBar_3->setWindowTitle(QApplication::translate("MainWindow", "toolBar_3", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOWGIFMOD_H

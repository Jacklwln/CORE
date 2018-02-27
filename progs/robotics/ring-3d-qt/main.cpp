/* **************************************
   File: main.cpp

   Description:
   This is the entry point for the running the SSS algorithm
   for a rod robot amidst a collection of polyhedrons (obstacles).
   The framework will ultimately support a variety of robots.

   NOTES:
   0. This file originated in the (3d) ball program done by
   	Surin Ahn (June 2013) for the
    2013 Siemens Science & Math Competition under the
	supervision of Professor Yap.  Surin was a
    Regional Finalist (one out of 100 nationwide) from New York.
    In 2014, she went to Princeton in Computer Science.
   1. See examples of running this program in the Makefile.
   2. Format of input environment: see README FILE
   3. Use WASD keys to rotate the scene
   4. In Fall of 2014, we started to reorgnized all the programs
   	to be consistent with our general SSS framework.

Author:
    Surin Ahn, Ching-Hsiang Hsu, Ziyao Wei, Yi-Jen Chiang and Chee Yap (Fall 2015)

Update:
    July. 09, 2017

   ************************************** */

#include <algorithm>
#include <string>
#include <vector>

#include <QApplication>
#include <QDir>

#include "main.h"

using namespace std;

SoftSubdivisionSearch* initializeSSS(Point3d start, Point3d goal, string workingDir) {
    ConfBox3d* root = new ConfBox3d(Point3d(boxWidth/2, boxWidth/2, boxWidth/2), boxWidth);
    ConfBox3d::r0 = R;

    parseConfigFile(root);
    //parseConfigFile(root, workingDir);

    return new SoftSubdivisionSearch(root, epsilon, searchType, start, goal, startRot, goalRot);
}

void run() {
    mw_out << "Run " << runCounter++ << ":\n";

    Timer t;
    // start timer
    t.start();

    if (sss) {
        delete(sss);
    }

    sss = initializeSSS(start, goal, workingDir);
    path = sss->softSubdivisionSearch();
    int ct = sss->getNumSubdivisions();
    noPath = path.empty();

    // stop timer
    t.stop();
    // print the elapsed time in millisec
    mw_out << ">>\tTime used " << t.getElapsedTimeInMilliSec() << " ms\n";
    mw_out << ">>\tExpanded " << ct << " times" << "\n";

    mw_out << ">>\n";
    if (!noPath) {
        mw_out << ">>\tPath found !\n";
    } else {
        mw_out << ">>\tNo Path !\n";
    }
    mw_out << ">>\n";
    mw_out << ">>\tSearch Strategy: ";
    if (searchType == RANDOM) {
        mw_out << "Random\n";
    }
    if (searchType == BFS) {
        mw_out << "BFS\n";
    }
    if (searchType == GREEDY) {
        mw_out << "Greedy\n";
    }
    if (searchType == BIGREEDY) {
        mw_out << "BIGreedy\n";
    }
    if (searchType == GREEDY_SIZE) {
        mw_out << "Greedy+Size\n";
    }
    if (searchType == BIGREEDY_SIZE) {
        mw_out << "Bi Greedy+Size\n";
    }
    if (searchType == VORONOI) {
        mw_out << "Voronoi\n";
    }
    if (searchType == BIVORONOI) {
        mw_out << "Bi-Voronoi\n";
    }

    mw_out << ">>\n";
    mw_out << ">>\ttotal Free boxes: " << sss->freeCount << "\n";
    mw_out << ">>\ttotal Stuck boxes: " << sss->stuckCount << "\n\n";
}

void logNonInteractiveRun(bool noPath) {
    // do something...
    cout << "Non Interactive Run of Disc Robot" << endl;
    if (noPath) {
        cout << "No Path Found!" << endl;
    } else {
        cout << "Path was Found!" << endl;
    }
}

void pwdWorkingDir(){
    bool foundFiles = false;
    workingDir = QDir::currentPath().toStdString();

    // Test if the build directory is ring-3d. If so,
    // the path to the current working directory will
    // include /ring-3d
    int indexOfDesiredDir = workingDir.rfind("/ring-3d-qt/");
    if (indexOfDesiredDir != (int)std::string::npos) {
        workingDir = workingDir.substr(0, indexOfDesiredDir + 5);

        // Set current working directory to ring-3d
        QDir::setCurrent(workingDir.c_str());

        foundFiles = true;
    }

    // Test if program was downloaded from Github, and is the build directory.
    // Downloading it from Github will result in the folder having the name
    // /ring-3d-master instead of /ring-3d
    if (!foundFiles &&
        (indexOfDesiredDir = workingDir.rfind("/ring-3d-qt/")) != (int)std::string::npos) {
        workingDir = workingDir.substr(0, indexOfDesiredDir + 12);

        // Set current working directory to /ring-3d-master
        QDir::setCurrent(workingDir.c_str());

        foundFiles = true;
    }

    // Test if a build directory (/build-ring-3d-...) was created. This directory
    // will reside in the same directory as /ring-3d or /ring-3d-qt
    if (!foundFiles &&
        (indexOfDesiredDir = workingDir.rfind("/build-ring-3d-qt")) != (int)std::string::npos) {
        QDir dir(workingDir.substr(0, indexOfDesiredDir).c_str());

        if (dir.exists("ring-3d-qt/ring-3d-qt.pro")) {                  // Test if /v2-links exists
            workingDir = workingDir.substr(0, indexOfDesiredDir) + "/ring-3d-qt";

            // Set current working directory to ring-3d
            QDir::setCurrent(workingDir.c_str());

            foundFiles = true;
        } else if (dir.exists("ring-3d-qt/ring-3d-qt.pro")) {  // Test if /ring-3d-qt exists
            workingDir = workingDir.substr(0, indexOfDesiredDir) + "/ring-3d-qt";

            // Set current working directory to ring-3d
            QDir::setCurrent(workingDir.c_str());

            foundFiles = true;
        }
    }

    // Neither /ring-3d nor /ring-3d-qt could be found
    if (!foundFiles) {
        std::cerr << std::endl << "!! WARNING !!\n"
        << "The program may not work correctly or at all because the folder "
        "containing the program's files cannot be found.\n"
        "Make sure that the program is inside of a folder named \"ring-3d-qt\".\n"
        "If it is not, rename the folder to \"ring-3d-qt\" before running again.\n";
    }
}

// MAIN PROGRAM: ========================================
int main(int argc, char* argv[]) {

    pwdWorkingDir();

    if (argc > 1) interactive = atoi(argv[1]);  // Interactive (0) or no (>0)
    if (argc > 2) start.setX(atof(argv[2]));    // start x
    if (argc > 3) start.setY(atof(argv[3]));    // start y
    if (argc > 4) start.setZ(atof(argv[4]));    // start z
    if (argc > 5) goal.setX(atof(argv[5]));    // goal x
    if (argc > 6) goal.setY(atof(argv[6]));    // goal y
    if (argc > 7) goal.setZ(atof(argv[7]));    // goal z
    if (argc > 8) epsilon = atof(argv[8]);    // epsilon (resolution)
    if (argc > 9) R       = atof(argv[9]);    // rod length
    if (argc > 10) fileName = argv[10];     // Input file name
    if (argc > 11) boxWidth = atof(argv[11]);  // boxWidth
    if (argc > 12) windowPosX = atoi(argv[12]);  // window X pos
    if (argc > 13) windowPosY = atoi(argv[13]);  // window Y pos
    if (argc > 14) searchType   = atoi(argv[14]);  // PriorityQ Type (random or no)
    if (argc > 15) seed   = static_cast<unsigned int>(atoi(argv[15]));    // for random number generator
    if (argc > 16) {
        inputDir  = argv[16];    // path for input files
        std::stringstream ss;
        ss << inputDir << "/" << fileName;  // create full file name
        defaultFilePath = ss.str();
    }
    if (argc > 17) deltaX  = atof(argv[17]);  // x-translation of input file
    if (argc > 18) deltaY  = atof(argv[18]);  // y-translation of input file
    if (argc > 19) deltaZ = atof(argv[19]);   // z-translation of input file
    if (argc > 20) scale  = atof(argv[20]);   // scaling of input file
    if (argc > 21) eye.setX(atof(argv[21]));
    if (argc > 22) eye.setY(atof(argv[22]));
    if (argc > 23) eye.setZ(atof(argv[23]));
    if (argc > 24) at.setX(atof(argv[24]));
    if (argc > 25) at.setY(atof(argv[25]));
    if (argc > 26) at.setZ(atof(argv[26]));
    if (argc > 27) up.setX(atof(argv[27]));
    if (argc > 28) up.setY(atof(argv[28]));
    if (argc > 29) up.setZ(atof(argv[29]));
    if (argc > 30) transparency = atoi(argv[30]);
    if (argc > 31) frameRate = atoi(argv[31]);
//    if (argc > 34) {
//        view_rotate_angles[0] = atof(argv[32]);
//        view_rotate_angles[1] = atof(argv[33]);
//        view_rotate_angles[2] = atof(argv[34]);
//    }

    if (interactive > 0) {  // non-interactive
        logNonInteractiveRun(noPath);
        return 0;
    }

    g_fptr = fopen("debug.txt", "w");
    //g_fptr = stderr;

    // Allow creation of Qt GUI
    QApplication app(argc, argv);
    parseCfgList();
    parseCfgFile();
    window = new MainWindow();    
    run();
    window->show();

    return app.exec();
}

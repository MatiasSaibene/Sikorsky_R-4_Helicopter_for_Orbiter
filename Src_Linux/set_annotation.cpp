#include "R4.h"
#include <cstddef>

void R4::SetAnnotation_Messages(bool show_help){

    const char *message1 = "";
    const char *message2 = "";
    const char *message3 = "";
    const char *message4 = "";
    const char *message5 = "";
    const char *message6 = "";
    const char *message7 = "";
    const char *message8 = "";
    const char *message9 = "";
    const char *message10 = "";
    const char *message11 = "";
    const char *message12 = "";
    const char *message13 = "";
    const char *message14 = "";
    const char *message15 = "";


    if(show_help == true){

        message1 = "";
        message2 = ""; 
        message3 = ""; 
        message4 = ""; 
        message5 = ""; 
        message6 = "E to toggle engine on/off";
        message7 = "+/- to throttle engine";
        message8 =  "NUMPAD 0 / . to set collective";
        message9 = "NUMPAD 1/3 to apply tailrotor torque";
        message10 = "A to engage altitude hold (controls collective)";
        message11 = "B to apply brakes, CTRL+B to apply/release parking brake";
        message12 = "CTRL+L to toggle exterior lights";
        message13 = "CTRL+(+/-) to change cabin light level";
        message14 = "SHIFT+(+/-) to change instrument light level";
        message15 = "Press I to hide help information";

    } else if(show_help == false){

        message1 = "";
        message2 = "";
        message3 = "";
        message4 = "";
        message5 = "";
        message6 = "";
        message7 = "";
        message8 = "";
        message9 = "";
        message10 = "";
        message11 = "";
        message12 = "";
        message13 = "";
        message14 = "";
        message15 = "";

    }

    oapiAnnotationSetText(message1_annotation, message1);
    oapiAnnotationSetText(message2_annotation, message2);
    oapiAnnotationSetText(message3_annotation, message3);
    oapiAnnotationSetText(message4_annotation, message4);
    oapiAnnotationSetText(message5_annotation, message5);
    oapiAnnotationSetText(message6_annotation, message6);
    oapiAnnotationSetText(message7_annotation, message7);
    oapiAnnotationSetText(message8_annotation, message8);
    oapiAnnotationSetText(message9_annotation, message9);
    oapiAnnotationSetText(message10_annotation, message10);
    oapiAnnotationSetText(message11_annotation, message11);
    oapiAnnotationSetText(message12_annotation, message12);
    oapiAnnotationSetText(message13_annotation, message13);
    oapiAnnotationSetText(message14_annotation, message14);
    oapiAnnotationSetText(message15_annotation, message15);

}
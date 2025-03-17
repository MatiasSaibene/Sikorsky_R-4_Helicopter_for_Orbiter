#include "R4.h"

#include <vector>
#include <cstring> // Para strcpy

void R4::SetAnnotation_Messages(bool show_help) {
    
    std::vector<char> message1(256), message2(256), message3(256);
    std::vector<char> message4(256), message5(256), message6(256);
    std::vector<char> message7(256), message8(256), message9(256);
    std::vector<char> message10(256), message11(256), message12(256);
    std::vector<char> message13(256), message14(256), message15(256);

    if (show_help) {
        strcpy(message6.data(), "E to toggle engine on/off");
        strcpy(message7.data(), "+/- to throttle engine");
        strcpy(message8.data(), "NUMPAD 0 / . to set collective");
        strcpy(message9.data(), "NUMPAD 1/3 to apply tailrotor torque");
        strcpy(message10.data(), "A to engage altitude hold (controls collective)");
        strcpy(message11.data(), "B to apply brakes, CTRL+B to apply/release parking brake");
        strcpy(message12.data(), "CTRL+L to toggle exterior lights");
        strcpy(message13.data(), "CTRL+(+/-) to change cabin light level");
        strcpy(message14.data(), "SHIFT+(+/-) to change instrument light level");
        strcpy(message15.data(), "Press I to hide help information");
    }

    oapiAnnotationSetText(message1_annotation, message1.data());
    oapiAnnotationSetText(message2_annotation, message2.data());
    oapiAnnotationSetText(message3_annotation, message3.data());
    oapiAnnotationSetText(message4_annotation, message4.data());
    oapiAnnotationSetText(message5_annotation, message5.data());
    oapiAnnotationSetText(message6_annotation, message6.data());
    oapiAnnotationSetText(message7_annotation, message7.data());
    oapiAnnotationSetText(message8_annotation, message8.data());
    oapiAnnotationSetText(message9_annotation, message9.data());
    oapiAnnotationSetText(message10_annotation, message10.data());
    oapiAnnotationSetText(message11_annotation, message11.data());
    oapiAnnotationSetText(message12_annotation, message12.data());
    oapiAnnotationSetText(message13_annotation, message13.data());
    oapiAnnotationSetText(message14_annotation, message14.data());
    oapiAnnotationSetText(message15_annotation, message15.data());
}

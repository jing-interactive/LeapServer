/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#include <iostream>
#include "../LeapSDK/include/Leap.h"
#include "../_common/ofxOsc/ofxOscSender.h"
#include <windows.h>

using namespace Leap;

ofxOscSender tuioSender;

// lerp() and lmap() are borrowed from /cinder/CinderMath.h
template<typename T, typename L>
T lerp( const T &a, const T &b, L factor )
{
    return a + ( b - a ) * factor;
}

template<typename T>
T lmap(T val, T inMin, T inMax, T outMin, T outMax)
{
    return outMin + (outMax - outMin) * ((val - inMin) / (inMax - inMin));
}

template<typename T>
T clamp( T inVal, T minVal, T maxVal )
{
    return (inVal < minVal) ? minVal : ((inVal > maxVal) ? maxVal : inVal);
}

template<typename T>
T normalize( T inVal, T minVal, T maxVal )
{
    return clamp<T>(lmap<T>(inVal, minVal, maxVal, 0.0f, 1.0f), 0.0f, 1.0f);
}

#ifdef INIT_BBOX
Vector bboxMin(+FLT_MAX, +FLT_MAX, +FLT_MAX);
Vector bboxMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
#else
Vector bboxMin(-180.0f, 100.0f, -350.0f);
Vector bboxMax(+180.0f, 450.0f, 100.0f);
#endif

class AppListener : public Listener 
{
public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);

    void processGesture( const Gesture& gesture, const Controller &controller ) 
    {
        switch (gesture.type()) 
        {
        case Gesture::TYPE_CIRCLE:
            {
                CircleGesture circle = gesture;
                std::string clockwiseness;

                if (circle.pointable().direction().angleTo(circle.normal()) <= PI/4)
                {
                    clockwiseness = "clockwise";
                } else
                {
                    clockwiseness = "counterclockwise";
                }

                // Calculate angle swept since last frame
                float sweptAngle = 0;
                if (circle.state() != Gesture::STATE_START) 
                {
                    CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
                    sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
                }
                std::cout << "Circle id: " << gesture.id()
                    << ", state: " << gesture.state()
                    << ", progress: " << circle.progress()
                    << ", radius: " << circle.radius()
                    << ", angle " << sweptAngle * RAD_TO_DEG
                    <<  ", " << clockwiseness << std::endl;
                break;
            }
        case Gesture::TYPE_SWIPE:
            {
                SwipeGesture swipe = gesture;
                std::cout << "Swipe id: " << gesture.id()
                    << ", state: " << gesture.state()
                    << ", direction: " << swipe.direction()
                    << ", speed: " << swipe.speed() << std::endl;
                break;
            }
        case Gesture::TYPE_KEY_TAP:
            {
                KeyTapGesture tap = gesture;
                std::cout << "Key Tap id: " << gesture.id()
                    << ", state: " << gesture.state()
                    << ", position: " << tap.position()
                    << ", direction: " << tap.direction()<< std::endl;
                break;
            }
        case Gesture::TYPE_SCREEN_TAP:
            {
                ScreenTapGesture screentap = gesture;
                std::cout << "Screen Tap id: " << gesture.id()
                    << ", state: " << gesture.state()
                    << ", position: " << screentap.position()
                    << ", direction: " << screentap.direction()<< std::endl;
                break;
            }
        default:
            //std::cout << "Unknown gesture type." << std::endl;
            break;
        }
    }

    void processHand(const Hand& hand) 
    {
        // Check if the hand has any fingers
        const FingerList& fingers = hand.fingers();
        if (fingers.empty()) 
            return;

        int32_t handId = hand.id();

#ifdef _DEBUG
        // Get the hand's sphere radius and palm position
        std::cout << "Hand #" << handId << " sphere radius: " << hand.sphereRadius()
            << " mm, palm position: " << hand.palmPosition() << std::endl;
#endif

        // Get the hand's normal vector and direction
        const Vector normal = hand.palmNormal();
        const Vector direction = hand.direction();

#if 0
        // Calculate the hand's pitch, roll, and yaw angles
        std::cout << "Hand pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
            << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
            << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << std::endl;
#endif

        for (int i = 0; i < fingers.count(); ++i)
        {
            const Vector& tipPos = fingers[i].tipPosition();
            const Vector& tipVel = fingers[i].tipVelocity();
#ifdef INIT_BBOX
            // max/min bbox
            bboxMax.x = std::max<float>(bboxMax.x, tipPos.x);
            bboxMax.y = std::max<float>(bboxMax.y, tipPos.y);
            bboxMax.z = std::max<float>(bboxMax.z, tipPos.z);

            bboxMin.x = std::min<float>(bboxMin.x, tipPos.x);
            bboxMin.y = std::min<float>(bboxMin.y, tipPos.y);
            bboxMin.z = std::min<float>(bboxMin.z, tipPos.z);
#endif
            Vector tuioPos;
            tuioPos.x = normalize(tipPos.x, bboxMin.x, bboxMax.x);
            tuioPos.y = normalize(tipPos.y, bboxMax.y, bboxMin.y);
            tuioPos.z = normalize(tipPos.z, bboxMin.z, bboxMax.z);

            // tuio
            {
                ofxOscMessage m;
                m.setAddress( "/tuio/2Dcur" );
                m.addStringArg("set");
                m.addIntArg(kFingerPerHand * handId + fingers[i].id());				// id
                m.addFloatArg(tuioPos.x);	// x
                m.addFloatArg(tuioPos.y);	// y
                // TOTO
                m.addFloatArg(tipVel.x / 400);			// dX
                m.addFloatArg(tipVel.y / 400);			// dY
                m.addFloatArg(0);		// maccel
                mBundle.addMessage(m);
                mAliveMsg.addIntArg(fingers[i].id());				// add blob to list of ALL active IDs
            }
        }
#ifdef INIT_BBOX            
        std::cout << "min: " << bboxMin << std::endl;
        std::cout << "max: " << bboxMax << std::endl;
#endif
    }

private:
    ofxOscBundle    mBundle;
    ofxOscMessage   mAliveMsg;
    Frame           mPrevFrame;
    static const int kFingerPerHand = 20; // 20 is big enough for TUIO id
};

void AppListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
}

void AppListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
    controller.setPolicyFlags(Controller::POLICY_BACKGROUND_FRAMES);
#if 0
    controller.enableGesture(Gesture::TYPE_CIRCLE);
    controller.enableGesture(Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Gesture::TYPE_SWIPE);
#endif
}

void AppListener::onDisconnect(const Controller& controller) {
    std::cout << "Disconnected" << std::endl;
}

void AppListener::onExit(const Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void AppListener::onFrame(const Controller& controller) 
{
    const Frame frame = controller.frame();
    if (mPrevFrame == frame)
        return;

    mPrevFrame = frame;

    const HandList&     hands       = frame.hands();
    const FingerList&   fingers     = frame.fingers();
    const ToolList&     tools       = frame.tools();
    const GestureList&  gestures    = frame.gestures();

    if (hands.empty() && fingers.empty() && tools.empty() && gestures.empty())
        return;

#if 0
    std::cout 
        << "Frame id: "     << frame.id()
        << ", timestamp: "  << frame.timestamp()
        << " hands: "       << hands.count()
        << " fingers: "     << fingers.count()
        << " tools: "       << tools.count()
        << " gestures: "    << gestures.count() 
        << std::endl;
#endif
    mBundle.clear();
    mAliveMsg.clear();
    {
        mAliveMsg.setAddress("/tuio/2Dcur");
        mAliveMsg.addStringArg("alive");
    }

    ofxOscMessage fseq;
    {
        fseq.setAddress( "/tuio/2Dcur" );
        fseq.addStringArg( "fseq" );
        fseq.addIntArg(static_cast<int32_t>(frame.id()));
    }

    for (int i = 0; i < hands.count(); ++i)
    {
        processHand(hands[i]);
    }

    mBundle.addMessage(mAliveMsg);
    mBundle.addMessage(fseq);
    tuioSender.sendBundle(mBundle);

    // Get gestures
    for (int i = 0; i < gestures.count(); ++i) 
    {
        processGesture(gestures[i], controller);
    }
}

int main() {

    tuioSender.setup("127.0.0.1", 3333);

    // Create a sample listener and controller
    AppListener listener;
    Controller mController;

    DeviceList devList = mController.devices();
    if (devList.isEmpty())
    {
        fprintf(stderr, "devList is empty!\n");
    }

    // Have the sample listener receive events from the controller
    mController.addListener(listener);

    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    // Remove the sample listener when done
    mController.removeListener(listener);

    return 0;
}

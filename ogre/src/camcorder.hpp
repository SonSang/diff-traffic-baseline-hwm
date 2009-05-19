/*
-----------------------------------------------------------------------------
Copyright (c) 2008 Torus Knot Software Ltd

You may use this sample code for anything you like, it is public domain.
-----------------------------------------------------------------------------
*/
/*
-----------------------------------------------------------------------------
Filename:    camcorder.hpp
Description: A subsystem for recording camera points and playing them back
-----------------------------------------------------------------------------
*/

#ifndef __camcorder_HPP__
#define __camcorder_HPP__

#include "Ogre.h"
#include "OIS/OIS.h"
#include "OgreTextAreaOverlayElement.h"

using namespace Ogre;


/** Helper class which provides an interface to the Camcorder */
class CamcorderHelper
{
public:
	CamcorderHelper();
	virtual ~CamcorderHelper();

	void init(Viewport* vp, Camera* cam);

	/** Process unbuffered keyboard state.
	*/
	void processUnbufferedKeyboard(OIS::Keyboard* keyboard, Real timeElapsed);

	/** Update the camera based on a frame.
	@remarks
		Will only update the camera if in playback mode, safe to
		call in all cases
	*/
	void update(Real timeElapsed);


	enum Mode
	{
		/// Not actively doing anything (although manual keyframes can be taken)
		IDLE,
		/// Recording the motion of the camera according to setKeyFrameFrequency
		RECORDING,
		/// Playing back an animation
		PLAYBACK
	};

	/// Get the current mode
	Mode getMode() const { return mMode; }
	/// Manually change the mode rather than let the input processing do it
	void setMode(Mode m);

	bool isPlayingBack() const;
	bool isRecording() const;
	bool isIdle() const;

	/// Enable the class, showing the overlay, processing input and recording / playback if required
	void setEnabled(bool enabled);
	bool getEnabled() const { return mEnabled; }

	/// Set the position interpolation mode
	void setPositionInterpolationMode(Animation::InterpolationMode m);
	/// Set the rotation interpolation mode
	void setRotationInterpolationMode(Animation::RotationInterpolationMode m);

	Animation::InterpolationMode getPositionInterpolationMode() const { return mInterpolationMode; }
	/// Set the rotation interpolation mode
	Animation::RotationInterpolationMode getRotationInterpolationMode() const { return mRotationInterpolationMode; }

	/// Set how often to take a keyframe in seconds when recording
	void setKeyFrameFrequency(Real f) { mKeyFrameFrequency = f; }
	/// Get how often to take a keyframe in seconds when recording
	Real getKeyFrameFrequency() const { return mKeyFrameFrequency; }


	/// Take a keyframe of the current camera position
	void addKeyFrame(Real time);


protected:
	Mode mMode;
	bool mEnabled;
	Viewport* mViewport;
	Camera* mCamera;
	Real mPlaybackTime;
	Real mRecordingTime;
	Real mLastKeyTime;

	Vector3 mSavedPos;
	Quaternion mSavedOrientation;
	Animation* mCurrentAnimation;
	NodeAnimationTrack* mCurrentTrack;

	Animation::InterpolationMode mInterpolationMode;
	Animation::RotationInterpolationMode mRotationInterpolationMode;
	Real mKeyFrameFrequency;
	Real mPlaybackSpeed;

	Overlay* mOverlay;
	OverlayElement* mModeText;
	OverlayElement* mLengthText;
	OverlayElement* mKeyFramesText;
	OverlayElement* mFreqText;
	OverlayElement* mFreqLabel;
	OverlayElement* mCurrentPosText;


	void createOverlay();
	TextAreaOverlayElement* createText(Real left, Real top, const String& text);


};

#endif

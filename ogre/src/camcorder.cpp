/*
-----------------------------------------------------------------------------
Copyright (c) 2000-2008 Torus Knot Software Ltd

You may use this code for anything you like, it is public domain.
-----------------------------------------------------------------------------
*/
#include "camcorder.hpp"
#include "OgreOverlayManager.h"
#include "OgreBorderPanelOverlayElement.h"

//---------------------------------------------------------------------
CamcorderHelper::CamcorderHelper()
: mMode(IDLE)
, mEnabled(false)
, mViewport(0)
, mCurrentAnimation(0)
, mCurrentTrack(0)
, mInterpolationMode(Animation::IM_SPLINE)
, mRotationInterpolationMode(Animation::RIM_SPHERICAL)
, mKeyFrameFrequency(3.0)
, mPlaybackSpeed(1.0)
, mOverlay(0)
{

}
//---------------------------------------------------------------------
CamcorderHelper::~CamcorderHelper()
{
	delete mCurrentAnimation;
}
//---------------------------------------------------------------------
void CamcorderHelper::setEnabled(bool enabled)
{
	mEnabled = enabled;

	if (!mOverlay)
		createOverlay();

	if (enabled)
		mOverlay->show();
	else
		mOverlay->hide();
}
//---------------------------------------------------------------------
void CamcorderHelper::setMode(Mode m)
{
	if (m != mMode)
	{
		switch (m)
		{
		case PLAYBACK:
			mSavedPos = mCamera->getDerivedPosition();
			mSavedOrientation = mCamera->getDerivedOrientation();
			mPlaybackTime = 0;
			break;
		case IDLE:
			if (mMode == PLAYBACK)
			{
				// return camera to saved position
				mCamera->setPosition(mSavedPos);
				mCamera->setOrientation(mSavedOrientation);
			}
			break;
		case RECORDING:
			mRecordingTime = 0;
			mLastKeyTime = 0;
			addKeyFrame(mLastKeyTime);
			break;
		};

		mMode = m;
	}


}
//---------------------------------------------------------------------
void CamcorderHelper::init(Viewport* vp, Camera* cam)
{
	mViewport = vp;
	mCamera = cam;
}
//---------------------------------------------------------------------
void CamcorderHelper::setPositionInterpolationMode(Animation::InterpolationMode m)
{
	mInterpolationMode = m;
	if (mCurrentAnimation)
		mCurrentAnimation->setInterpolationMode(m);
}
//---------------------------------------------------------------------
void CamcorderHelper::setRotationInterpolationMode(Animation::RotationInterpolationMode m)
{
	mRotationInterpolationMode = m;
	if (mCurrentAnimation)
		mCurrentAnimation->setRotationInterpolationMode(m);
}
//---------------------------------------------------------------------
void CamcorderHelper::processUnbufferedKeyboard(OIS::Keyboard* keyboard, Real timeElapsed)
{
	static float delay = 0;

	delay -= timeElapsed;

	if (!mEnabled)
		return;

	// Toggle playback mode
	if (keyboard->isKeyDown(OIS::KC_RETURN) && delay <= 0)
	{
		if (mMode == IDLE)
		{
			setMode(PLAYBACK);
		}
		else if (mMode == PLAYBACK)
		{
			setMode(IDLE);
		}

		delay = 0.5;
	}

	if (keyboard->isKeyDown(OIS::KC_INSERT) && delay <= 0)
	{
		if (mMode == IDLE)
		{
			setMode(RECORDING);
		}
		else if (mMode == RECORDING)
		{
			setMode(IDLE);
		}

		delay = 0.5;
	}

	if (keyboard->isKeyDown(OIS::KC_SPACE) && delay <= 0)
	{
		// take snapshot
		if (mMode == IDLE)
		{
			if (!mCurrentAnimation)
				mLastKeyTime = 0.0f;
			else
				mLastKeyTime += mKeyFrameFrequency;

			addKeyFrame(mLastKeyTime);

			delay = 0.5;
		}

	}
	if (keyboard->isKeyDown(OIS::KC_LBRACKET) && delay <= 0)
	{
		if (mMode == PLAYBACK)
		{
			mPlaybackSpeed -= 0.1;
			if (mPlaybackSpeed < 0.1)
				mPlaybackSpeed = 0.1;
		}
		else
		{
			mKeyFrameFrequency -= 0.1;
			if (mKeyFrameFrequency < 0.1)
				mKeyFrameFrequency = 0.1;
		}
		delay = 0.1;
	}
	if (keyboard->isKeyDown(OIS::KC_RBRACKET) && delay <= 0)
	{
		if (mMode == PLAYBACK)
		{
			mPlaybackSpeed += 0.1;
		}
		else
		{
			mKeyFrameFrequency += 0.1;
		}
		delay = 0.1;
	}

}
//---------------------------------------------------------------------
void CamcorderHelper::addKeyFrame(Real time)
{
	// take a snapshot
	if (!mCurrentAnimation)
	{
		mCurrentAnimation = new Animation("CamCorderAnim", 0);
		mCurrentTrack = mCurrentAnimation->createNodeTrack(0);
		mCurrentAnimation->setInterpolationMode(mInterpolationMode);
		mCurrentAnimation->setRotationInterpolationMode(mRotationInterpolationMode);
		mCurrentTrack->setUseShortestRotationPath(false);
	}

	if (time == 0)
		mCurrentTrack->removeAllKeyFrames();

	TransformKeyFrame* kf = mCurrentTrack->createNodeKeyFrame(time);
	mCurrentAnimation->setLength(time);

	kf->setTranslate(mCamera->getDerivedPosition());
	kf->setRotation(mCamera->getDerivedOrientation());


}
//---------------------------------------------------------------------
void CamcorderHelper::update(Real timeElapsed)
{
	if (!mEnabled)
		return;

	if (mMode == PLAYBACK && mCurrentTrack)
	{
		mPlaybackTime += timeElapsed * mPlaybackSpeed;

		TransformKeyFrame kf(0, 0);
		mCurrentTrack->getInterpolatedKeyFrame(TimeIndex(mPlaybackTime), &kf);

		mCamera->setPosition(kf.getTranslate());
		mCamera->setOrientation(kf.getRotation());

		mCurrentPosText->setCaption(StringConverter::toString(mPlaybackTime));
		mModeText->setCaption("PLAY");
		mModeText->setColour(ColourValue::Green);

		mFreqText->setCaption(StringConverter::toString(mPlaybackSpeed));
		mFreqLabel->setCaption("Playback Speed:");


	}
	else if (mMode == RECORDING)
	{
		mRecordingTime += timeElapsed;
		if (mRecordingTime - mLastKeyTime >= mKeyFrameFrequency)
		{
			mLastKeyTime = mRecordingTime;
			addKeyFrame(mLastKeyTime);
		}

		mCurrentPosText->setCaption(StringConverter::toString(mRecordingTime));
		mModeText->setCaption("REC");
		mModeText->setColour(ColourValue::Red);
		mFreqText->setCaption(StringConverter::toString(mKeyFrameFrequency));
		mFreqLabel->setCaption("Recording Freq:");

	}
	else
	{
		mModeText->setCaption("IDLE");
		mModeText->setColour(ColourValue::White);
		mCurrentPosText->setCaption("");
		mFreqText->setCaption(StringConverter::toString(mKeyFrameFrequency));
		mFreqLabel->setCaption("Recording Freq:");
	}

	if (mCurrentAnimation)
	{
		mLengthText->setCaption(StringConverter::toString(mCurrentAnimation->getLength()));
		mKeyFramesText->setCaption(StringConverter::toString(mCurrentTrack->getNumKeyFrames()));
	}
	else
	{
		mLengthText->setCaption("0");
		mKeyFramesText->setCaption("0");
	}






}
//---------------------------------------------------------------------
bool CamcorderHelper::isPlayingBack() const
{
	return mMode == PLAYBACK;
}
//---------------------------------------------------------------------
bool CamcorderHelper::isRecording() const
{
	return mMode == RECORDING;
}
//---------------------------------------------------------------------
bool CamcorderHelper::isIdle() const
{
	return mMode == IDLE;
}
//---------------------------------------------------------------------
#define OVERLAY_WIDTH 400
#define OVERLAY_HEIGHT 85
void CamcorderHelper::createOverlay()
{
	OverlayManager& omgr = OverlayManager::getSingleton();
	mOverlay = omgr.create("CamCorder");

	BorderPanelOverlayElement* cnt = static_cast<BorderPanelOverlayElement*>(
		omgr.createOverlayElement("BorderPanel", "CC/Main"));

	cnt->setMetricsMode(GMM_PIXELS);
	cnt->setVerticalAlignment(GVA_BOTTOM);
	cnt->setHorizontalAlignment(GHA_CENTER);
	cnt->setPosition(-OVERLAY_WIDTH / 2, -(OVERLAY_HEIGHT + 5));
	cnt->setDimensions(OVERLAY_WIDTH, OVERLAY_HEIGHT);
	cnt->setMaterialName("Core/StatsBlockCenter");
	cnt->setBorderSize(1, 1, 1, 1);
	cnt->setBorderMaterialName("Core/StatsBlockBorder");
	cnt->setTopLeftBorderUV(0.0000, 1.0000, 0.0039, 0.9961);
	cnt->setTopBorderUV(0.0039, 1.0000, 0.9961, 0.9961);
	cnt->setTopRightBorderUV(0.9961, 1.0000, 1.0000, 0.9961);
	cnt->setLeftBorderUV(0.0000, 0.9961, 0.0039, 0.0039);
	cnt->setRightBorderUV(0.9961, 0.9961, 1.0000, 0.0039);
	cnt->setBottomLeftBorderUV(0.0000, 0.0039, 0.0039, 0.0000);
	cnt->setBottomBorderUV(0.0039, 0.0039, 0.9961, 0.0000);
	cnt->setBottomRightBorderUV(0.9961, 0.0039, 1.0000, 0.0000);

	mOverlay->add2D(cnt);

	TextAreaOverlayElement* label = createText(5, 5, "Mode:");
	cnt->addChild(label);
	label = createText(5, 20, "Animation Len:");
	cnt->addChild(label);
	label = createText(5, 35, "# Keyframes:");
	cnt->addChild(label);
	mFreqLabel = createText(5, 50, "Keyframe Freq:");
	cnt->addChild(mFreqLabel);
	label = createText(5, 65, "Current Pos:");
	cnt->addChild(label);

	label = createText(200, 5, "INSERT = record");
	cnt->addChild(label);
	label = createText(200, 20, "RETURN = play");
	cnt->addChild(label);
	label = createText(200, 35, "SPACE = manual keyframe");
	cnt->addChild(label);
	label = createText(200, 50, "[] = change freq / playback");
	cnt->addChild(label);


	mModeText = createText(100, 5, "");
	cnt->addChild(mModeText);
	mLengthText = createText(100, 20, "");
	cnt->addChild(mLengthText);
	mKeyFramesText = createText(100, 35, "");
	cnt->addChild(mKeyFramesText);
	mFreqText = createText(100, 50, "");
	cnt->addChild(mFreqText);
	mCurrentPosText = createText(100, 65, "");
	cnt->addChild(mCurrentPosText);


	mOverlay->hide();


}
//---------------------------------------------------------------------
TextAreaOverlayElement* CamcorderHelper::createText(Real left, Real top, const String& text)
{
	OverlayManager& omgr = OverlayManager::getSingleton();
	static int counter = 0;
	TextAreaOverlayElement* t = static_cast<TextAreaOverlayElement*>(
		omgr.createOverlayElement("TextArea", "CC/ModeLabel/" + StringConverter::toString(counter++)));
	t->setMetricsMode(GMM_PIXELS);
	t->setVerticalAlignment(GVA_TOP);
	t->setHorizontalAlignment(GHA_LEFT);
	t->setPosition(left, top);
	t->setDimensions(30, text.length() * 20);
	t->setCaption(text);
	t->setFontName("BlueHighway");
	t->setCharHeight(16);

	return t;

}

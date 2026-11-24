#include "stdafx.h"

#include "CameraFirstEye.h"
#include "xr_level_controller.h"
#include "..\xr_3da\xr_object.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCameraFirstEye::CCameraFirstEye(CObject* p, u32 flags) : CCameraBase(p, flags) {}

CCameraFirstEye::~CCameraFirstEye() {}

void CCameraFirstEye::Load(LPCSTR section)
{
    inherited::Load(section);
    style = csFirstEye;
    ResetFreeAimCamera();
}

void CCameraFirstEye::ResetFreeAimCamera()
{
    m_fFreeAimRoll = 0.f;
    m_fTargetFreeAimRoll = 0.f;
}

void CCameraFirstEye::Update(Fvector& point, Fvector& noise_dangle)
{
    const float dt = Device.fTimeDelta;
    const float rollAlpha = std::clamp(1.f - expf(-12.0f * dt), 0.f, 1.f);
    m_fFreeAimRoll += (m_fTargetFreeAimRoll - m_fFreeAimRoll) * rollAlpha;
    if (_abs(m_fFreeAimRoll) < 0.00001f && fis_zero(m_fTargetFreeAimRoll))
        m_fFreeAimRoll = 0.f;

    Fmatrix mR, R;
    Fmatrix rX, rY, rZ;
    rX.rotateX(noise_dangle.x);
    rY.rotateY(-noise_dangle.y);
    rZ.rotateZ(noise_dangle.z);
    R.mul_43(rY, rX);
    R.mulB_43(rZ);

    mR.identity();
    Fquaternion Q;
    Q.rotationYawPitchRoll(roll + m_fFreeAimRoll, yaw, pitch);
    mR.rotation(Q);
    mR.transpose();
    mR.mulB_43(R);

    vDirection.set(mR.k);
    vNormal.set(mR.j);

    if (m_Flags.is(flRelativeLink))
    {
        parent->XFORM().transform_dir(vDirection);
        parent->XFORM().transform_dir(vNormal);
    }

    vPosition.set(point);
}

void CCameraFirstEye::Move(int cmd, float val, float factor)
{
    if (bClampPitch)
    {
        while (pitch < lim_pitch[0])
            pitch += PI_MUL_2;
        while (pitch > lim_pitch[1])
            pitch -= PI_MUL_2;
    }

    switch (cmd)
    {
    case kDOWN: pitch -= val ? val : (rot_speed.y * Device.fTimeDelta / factor); break;
    case kUP: pitch += val ? val : (rot_speed.y * Device.fTimeDelta / factor); break;
    case kLEFT: yaw -= val ? val : (rot_speed.x * Device.fTimeDelta / factor); break;
    case kRIGHT: yaw += val ? val : (rot_speed.x * Device.fTimeDelta / factor); break;
    }

    if (bClampYaw)
        clamp(yaw, lim_yaw[0], lim_yaw[1]);
    if (bClampPitch)
        clamp(pitch, lim_pitch[0], lim_pitch[1]);
}

void CCameraFirstEye::OnActivate(CCameraBase* old_cam)
{
    ResetFreeAimCamera();
    if (old_cam && (m_Flags.is(flRelativeLink) == old_cam->m_Flags.is(flRelativeLink)))
        yaw = (old_cam)->yaw;
}
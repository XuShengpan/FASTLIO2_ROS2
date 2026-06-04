#include "ieskf.h"

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V21D& delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

V21D State::operator-(const State& other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;

    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;

    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    ////std::cout<<"IESKF::predict"<<std::endl;

    V3D omg_x_dt = (inp.gyro - m_x.bg) * dt;
    M3D Jr_dt = Jr(omg_x_dt) * dt;
    M3D Idt = M3D::Identity() * dt;
    M3D Rdt = m_x.r_wi * dt;

    m_dx.setZero();
    m_dx.segment<3>(0) = omg_x_dt;
    m_dx.segment<3>(3) = m_x.v * dt;
    m_dx.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-omg_x_dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr_dt;
    m_F.block<3, 3>(3, 12) = Idt;
    m_F.block<3, 3>(12, 0) = -Rdt* Sophus::SO3d::hat(inp.acc - m_x.ba);
    m_F.block<3, 3>(12, 18) = -Rdt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr_dt;
    m_G.block<3, 3>(12, 3) = -Rdt;
    m_G.block<3, 3>(15, 6) = Idt;
    m_G.block<3, 3>(18, 9) = Idt;

    m_x += m_dx;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::update()
{
    V21D b;
    M21D H, J, L;

    m_dx.setZero();
    H.setIdentity();
    m_shared_data.reset();

    State predict_x = m_x;

    M21D P_inv = m_P.inverse();

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, m_shared_data);
        if (!m_shared_data.valid) {
            //std::cout<<"shared_data is invalid !"<<std::endl;
            break;
        }

        H.setZero();
        b.setZero();

        m_dx = m_x - predict_x;
        J.setIdentity();
        J.block<3, 3>(0, 0) = JrInv(m_dx.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(m_dx.segment<3>(6));
        H += J.transpose() * P_inv * J;
        b += J.transpose() * P_inv * m_dx;

        H.block<12, 12>(0, 0) += m_shared_data.H;
        b.block<12, 1>(0, 0) += m_shared_data.b;

        m_dx = -H.inverse() * b;

        m_x += m_dx;
        m_shared_data.iter_num += 1;

        if (m_stop_func(m_dx))
            break;
    }

    L.setIdentity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Jr(m_dx.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(m_dx.segment<3>(6));
    m_P = L * H.inverse() * L.transpose();

}
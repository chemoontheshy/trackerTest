#include <iostream>
#include <list>
#include<vector>
#include <Eigen/Eigen>
#include <time.h>
#include<cmath>
using namespace std;
using position_type = Eigen::Vector2f;

/// <summary>�ٶ��������ͣ���ʾ[x, y]���������ϵ��ٶȷ�������λΪkm/h</summary>
using speed_type = Eigen::Vector2f;

/// <summary>��ĳ��Ŀ�����δ����⵽ʱ�������������Ϣ�����ʱ��������Ϊ��λ���˺�����Ϣ����ɾ��</summary>
int                     Keep = 30;

/// <summary>�ٶ���ǰ������Ŀ�������ٶȣ���λΪkm/h����Ӱ�쵽ƥ��ʱ��������Χ</summary>
float                   MaxSpeed = 100.f;

// ���ٶ�������ģ
auto __absolutely_speed = [](const speed_type& speed) -> float {
	return speed.norm();
};

// ͨ���ٶ����������˶�����
auto __speed_to_direction = [](const speed_type& speed) -> float {
	//auto m_r = speed.norm();
	//auto d_s = speed.y() / m_r;
	auto m_f = acosf(speed.y() / speed.norm())* 180.0 / M_PI;
	//��һ��������
	//speed.x() >= 0 ? m_f : 360-m_f;
	return speed.x() >= 0 ? m_f : 360 - m_f;
	//if (speed.x() >= 0) {
	//	return  m_f;
	//}
	////�ڶ���������
	//else if (speed.x() < 0 ) {
	//	return  360 - m_f;
	//}
	
};

int64_t __get_receipt(const int64_t epoch, const int source) noexcept
{
	auto temp_epch = epoch;
	int64_t receipt = 0;
	
	while (temp_epch) {
		receipt = receipt * 10 + temp_epch % 10;
		temp_epch /= 10;
	}
	receipt += source;
	return receipt;
}

typedef struct Range
{
	/// <summary>��Χ�����Ͻ����꣨����</summary>
	position_type           TopLeft;
	/// <summary>��Χ�����½����꣨����</summary>
	position_type           BottomRight;
}Range;



class TrackerImpl {
public:

	/// <summary>
	/// ���캯��
	/// </summary>
	/// <param name="configuration">���ò���</param>
	explicit TrackerImpl();

	/// <summary>
	/// ��������
	/// </summary>
	~TrackerImpl() noexcept;
	/// <summary>
	/// �����¼�⵽��Ŀ���λ�û�ȡ����ƥ���Ŀ���������Χ
	/// </summary>
	/// <param name="position">�¼�⵽��Ŀ���λ��</param>
	/// <returns>����ƥ���Ŀ���������Χ</returns>
	Range                              __GetSearchRange(const position_type& position) const noexcept;
	
private:
	/// <summary>��ĳ��Ŀ�����δ����⵽ʱ�������������Ϣ�����ʱ��������Ϊ��λ���˺�����Ϣ����ɾ��</summary>
	const int                          m_nKeep = Keep;
	/// <summary>�ٶ���ǰ������Ŀ�������ٶȣ���λΪkm/h����Ӱ�쵽ƥ��ʱ��������Χ</summary>
	const float                        m_fMaxSpeed = MaxSpeed;
};

TrackerImpl::TrackerImpl() {

}

TrackerImpl::~TrackerImpl() {

}

Range TrackerImpl::__GetSearchRange(const position_type& position) const noexcept
{
	Range range;
	float m_r = m_nKeep * (m_fMaxSpeed * 1000 / 3600);
	cout << " m_r" << m_r << endl;
	range.TopLeft.x() = position.x() - m_r;
	range.TopLeft.y() = position.y() + m_r;
	range.BottomRight.x() = position.x() + m_r;
	range.BottomRight.y() = position.x() - m_r;
	return range;
}

/// <summary>
/// ������
/// </summary>
/// <returns></returns>
int main()
{
	//����һ������һ��ֵ
	cout << "hello world" << endl;
	time_t myt = time(NULL);
	//1624259192
	int64_t epoch = 1624259192;
	int64_t r_epoch = 2919524261;
	int source = 34;
	cout << "m_r_epoch: " << r_epoch + source << endl;
	cout << "t_r_epoch: " << __get_receipt(epoch,source)<< endl;

	//������
	speed_type  m_speed;
	m_speed.x() = 3;
	m_speed.y() = 4;
	cout << "--------------------------------" << endl;
	cout << "�ٶ�v��������ģ��"<< __absolutely_speed(m_speed) << endl;

	//������
	speed_type  m_d_speed;
	m_d_speed.x() = 1;
	m_d_speed.y() = 2;
	cout << "--------------------------------" << endl;
	cout << "�˶�����" << __speed_to_direction(m_d_speed) << endl;

	//������
	position_type m_point(300, 400);
	TrackerImpl m_impl;
	Range m_range = m_impl.__GetSearchRange(m_point);
	printf("m_range.TopLeft(%f,%f)\n", m_range.TopLeft.x(), m_range.TopLeft.y());
	printf("m_range.BottomRight(%f,%f)\n", m_range.BottomRight.x(), m_range.BottomRight.y());

}
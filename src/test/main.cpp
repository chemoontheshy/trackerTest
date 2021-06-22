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
/// <summary>���ƥ����룬����Ϊ��λ��������Ŀ��ľ���С�ڸ�ֵʱ�Ż����ƥ��</summary>
float                   MaxMatchDistance = 20.f;

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

/// <summary>
/// ���ٶ��������
/// </summary>
typedef struct Feature
{
	position_type Position;
} Feature;

/// <summary>
/// Ŀ��״̬
/// </summary>
typedef struct State
{
	/// <summary>λ��</summary>
	position_type Position;
	/// <summary>�ٶ�</summary>
	speed_type    Speed;
} State;

/// <summary>
		/// �Ѵ��ڵ�Ŀ��
		/// </summary>
typedef struct Existence
{
	/// <summary>Ԥ����Ϣ</summary>
	State                         Predicted;
} Existence;

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
	/// <summary>
	/// ��existences�й��˵�����Ԥ������������Ŀ����붼���������Ŀ��
	/// </summary>
	/// <param name="newly">���룬��Ŀ��</param>
	/// <param name="existences">�������������Ŀ�꣬��������Ŀ���Ԥ����붼����Ļᱻֱ���޳�</param>
	/// <returns>���޳���Ŀ����</returns>
	std::size_t                        __Filter(const std::vector<Feature>& newly,
		std::list<Existence>& existences) const noexcept;
	
private:
	/// <summary>��ĳ��Ŀ�����δ����⵽ʱ�������������Ϣ�����ʱ��������Ϊ��λ���˺�����Ϣ����ɾ��</summary>
	const int                          m_nKeep = Keep;
	/// <summary>�ٶ���ǰ������Ŀ�������ٶȣ���λΪkm/h����Ӱ�쵽ƥ��ʱ��������Χ</summary>
	const float                        m_fMaxSpeed = MaxSpeed;
	/// <summary>���ƥ����룬����Ϊ��λ��������Ŀ��ľ���С�ڸ�ֵʱ�Ż����ƥ��</summary>
	const float                        m_fMaxMatchDistance = MaxMatchDistance;
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

std::size_t TrackerImpl::__Filter(const std::vector<Feature>& newly,
	std::list<Existence>& existences) const noexcept
{
	std::list<Existence> temp_existences;
	for (auto i : existences) {
		bool flag = false;
		for (int j = 0; j < newly.size(); j++)
		{
			auto m_distance = sqrt((newly.at(j).Position.x() - i.Predicted.Position.x()) * (newly.at(j).Position.x() - i.Predicted.Position.x()) 
				+ (newly.at(j).Position.y() - i.Predicted.Position.y()) * (newly.at(j).Position.y() - i.Predicted.Position.y()));
			cout << "m_distance: "<<m_distance << endl;
			if (m_distance < m_fMaxMatchDistance) {
				flag = true;
				break;
			}
		}
		if (flag) {
			temp_existences.push_back(i);
		}
	}
	std::size_t m_size = existences.size() - temp_existences.size();
	cout << "����ǰ: " << existences.size() << endl;
	//-----
	for (auto i : existences) {
		printf("point(%f,%f)\n", i.Predicted.Position.x(), i.Predicted.Position.y());
	}
	existences.swap(temp_existences);
	cout << "������: " << existences.size() << endl;
	for (auto i : existences) {
		printf("point(%f,%f)\n", i.Predicted.Position.x(), i.Predicted.Position.y());
	}
	  
	return m_size;
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
	int64_t epoch = 1624259194;
	int64_t r_epoch = 4919524261;
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

	//������
	std::vector<Feature> newly;
	Feature feature1;
	feature1.Position.x() = 100;
	feature1.Position.y() = 100;
	Feature feature2;
	feature2.Position.x() = 110;
	feature2.Position.y() = 110;
	Feature feature3;
	feature3.Position.x() = 110;
	feature3.Position.y() = 120;
	Feature feature4;
	feature4.Position.x() = 120;
	feature4.Position.y() = 120;

	newly.push_back(feature1);
	newly.push_back(feature2);
	newly.push_back(feature3);
	newly.push_back(feature4);

	std::list<Existence> existences;
	Existence existence1;
	existence1.Predicted.Position.x() = 110;
	existence1.Predicted.Position.y() = 130;
	Existence existence2;
	existence2.Predicted.Position.x() = 110;
	existence2.Predicted.Position.y() = 120;
	Existence existence3;
	existence3.Predicted.Position.x() = 120;
	existence3.Predicted.Position.y() = 130;
	Existence existence4;
	existence4.Predicted.Position.x() = 130;
	existence4.Predicted.Position.y() = 130;
	Existence existence5;
	existence5.Predicted.Position.x() = 150;
	existence5.Predicted.Position.y() = 150;
	Existence existence6;
	existence6.Predicted.Position.x() = 160;
	existence6.Predicted.Position.y() = 160;

	existences.push_back(existence1);
	existences.push_back(existence2);
	existences.push_back(existence3);
	existences.push_back(existence4);
	existences.push_back(existence5);
	existences.push_back(existence6);
	m_impl.__Filter(newly, existences);

}
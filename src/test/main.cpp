#include <iostream>
#include <list>
#include<vector>
#include <Eigen/Eigen>
#include <time.h>
#include<cmath>
using namespace std;
using position_type = Eigen::Vector2f;

/// <summary>速度向量类型，表示[x, y]两个方向上的速度分量，单位为km/h</summary>
using speed_type = Eigen::Vector2f;

/// <summary>当某个目标持续未被检测到时，程序持有其信息的最大时长，以秒为单位，此后其信息将被删除</summary>
int                     Keep = 30;

/// <summary>假定当前场景下目标的最大速度，单位为km/h。它影响到匹配时的搜索范围</summary>
float                   MaxSpeed = 100.f;

// 求速度向量的模
auto __absolutely_speed = [](const speed_type& speed) -> float {
	return speed.norm();
};

// 通过速度向量计算运动方向
auto __speed_to_direction = [](const speed_type& speed) -> float {
	//auto m_r = speed.norm();
	//auto d_s = speed.y() / m_r;
	auto m_f = acosf(speed.y() / speed.norm())* 180.0 / M_PI;
	//第一、四象限
	//speed.x() >= 0 ? m_f : 360-m_f;
	return speed.x() >= 0 ? m_f : 360 - m_f;
	//if (speed.x() >= 0) {
	//	return  m_f;
	//}
	////第二、三象限
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
	/// <summary>范围的左上角坐标（含）</summary>
	position_type           TopLeft;
	/// <summary>范围的右下角坐标（含）</summary>
	position_type           BottomRight;
}Range;



class TrackerImpl {
public:

	/// <summary>
	/// 构造函数
	/// </summary>
	/// <param name="configuration">配置参数</param>
	explicit TrackerImpl();

	/// <summary>
	/// 析构函数
	/// </summary>
	~TrackerImpl() noexcept;
	/// <summary>
	/// 根据新检测到的目标的位置获取用以匹配的目标的搜索范围
	/// </summary>
	/// <param name="position">新检测到的目标的位置</param>
	/// <returns>用以匹配的目标的搜索范围</returns>
	Range                              __GetSearchRange(const position_type& position) const noexcept;
	
private:
	/// <summary>当某个目标持续未被检测到时，程序持有其信息的最大时长，以秒为单位，此后其信息将被删除</summary>
	const int                          m_nKeep = Keep;
	/// <summary>假定当前场景下目标的最大速度，单位为km/h。它影响到匹配时的搜索范围</summary>
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
/// 主函数
/// </summary>
/// <returns></returns>
int main()
{
	//函数一，返回一个值
	cout << "hello world" << endl;
	time_t myt = time(NULL);
	//1624259192
	int64_t epoch = 1624259192;
	int64_t r_epoch = 2919524261;
	int source = 34;
	cout << "m_r_epoch: " << r_epoch + source << endl;
	cout << "t_r_epoch: " << __get_receipt(epoch,source)<< endl;

	//函数二
	speed_type  m_speed;
	m_speed.x() = 3;
	m_speed.y() = 4;
	cout << "--------------------------------" << endl;
	cout << "速度v的向量的模："<< __absolutely_speed(m_speed) << endl;

	//函数三
	speed_type  m_d_speed;
	m_d_speed.x() = 1;
	m_d_speed.y() = 2;
	cout << "--------------------------------" << endl;
	cout << "运动方向：" << __speed_to_direction(m_d_speed) << endl;

	//函数四
	position_type m_point(300, 400);
	TrackerImpl m_impl;
	Range m_range = m_impl.__GetSearchRange(m_point);
	printf("m_range.TopLeft(%f,%f)\n", m_range.TopLeft.x(), m_range.TopLeft.y());
	printf("m_range.BottomRight(%f,%f)\n", m_range.BottomRight.x(), m_range.BottomRight.y());

}
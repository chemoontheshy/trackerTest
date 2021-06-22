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
/// <summary>最大匹配距离，以米为单位，当两个目标的距离小于该值时才会进行匹配</summary>
float                   MaxMatchDistance = 20.f;

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

/// <summary>
/// 跟踪对象的特征
/// </summary>
typedef struct Feature
{
	position_type Position;
} Feature;

/// <summary>
/// 目标状态
/// </summary>
typedef struct State
{
	/// <summary>位置</summary>
	position_type Position;
	/// <summary>速度</summary>
	speed_type    Speed;
} State;

/// <summary>
		/// 已存在的目标
		/// </summary>
typedef struct Existence
{
	/// <summary>预测信息</summary>
	State                         Predicted;
} Existence;

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
	/// <summary>
	/// 从existences中过滤掉经过预测后距离所有新目标距离都过大的现有目标
	/// </summary>
	/// <param name="newly">输入，新目标</param>
	/// <param name="existences">输入输出，现有目标，与所有新目标的预测距离都过大的会被直接剔除</param>
	/// <returns>被剔除的目标数</returns>
	std::size_t                        __Filter(const std::vector<Feature>& newly,
		std::list<Existence>& existences) const noexcept;
	
private:
	/// <summary>当某个目标持续未被检测到时，程序持有其信息的最大时长，以秒为单位，此后其信息将被删除</summary>
	const int                          m_nKeep = Keep;
	/// <summary>假定当前场景下目标的最大速度，单位为km/h。它影响到匹配时的搜索范围</summary>
	const float                        m_fMaxSpeed = MaxSpeed;
	/// <summary>最大匹配距离，以米为单位，当两个目标的距离小于该值时才会进行匹配</summary>
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
	cout << "交换前: " << existences.size() << endl;
	//-----
	for (auto i : existences) {
		printf("point(%f,%f)\n", i.Predicted.Position.x(), i.Predicted.Position.y());
	}
	existences.swap(temp_existences);
	cout << "交换后: " << existences.size() << endl;
	for (auto i : existences) {
		printf("point(%f,%f)\n", i.Predicted.Position.x(), i.Predicted.Position.y());
	}
	  
	return m_size;
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
	int64_t epoch = 1624259194;
	int64_t r_epoch = 4919524261;
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

	//函数五
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
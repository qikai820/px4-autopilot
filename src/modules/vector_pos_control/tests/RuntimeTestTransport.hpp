#pragma once
#include <uORB/uORB.h>

template<typename T> struct Publisher {
	orb_id_t topic;
	orb_advert_t handle{nullptr};
	void publish(const T &data)
	{
		if (handle) { orb_publish(topic, handle, &data); }

		else { handle = orb_advertise(topic, &data); }
	}
	~Publisher() { if (handle) { orb_unadvertise(handle); } }
};
struct Subscription {
	orb_id_t topic;
	int fd;
	explicit Subscription(orb_id_t id) : topic(id), fd(orb_subscribe(id)) {}
	template<typename T> bool copy(T *data) { return orb_copy(topic, fd, data) == 0; }
	~Subscription() { orb_unsubscribe(fd); }
};

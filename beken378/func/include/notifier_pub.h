// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __NOTIFIER_H_
#define __NOTIFIER_H_


typedef void (*notify_func)(void *, int type, int value);

struct notifier {
    struct notifier *next;
    notify_func	    func;
    void	    	*arg;
};

struct notifier_req {
    notify_func	    func;
    void	    	*arg;
};

enum {
    SYS_EVENT = 0,
    WLAN_EVENT,
    POWER_EVENT,
};

extern struct notifier *wlan_evt_notifer;

int add_notifier(struct notifier **notif,  notify_func func, void *arg);
void remove_notifier(struct notifier **notif,  notify_func func, void *arg);
void notify(struct notifier *notif, int type, int val);

int register_wlan_notifier(notify_func func, void *arg);
void remove_wlan_notifier(notify_func func, void *arg);

#endif //__NOTIFIER_H_

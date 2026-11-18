#include "service/navigation/RgbcSampleStore.hpp"

namespace minicar::service {

void RgbcSampleStore::update(const model::NavigationSample& sample)
{
    if (sample.type != model::NavigationSampleType::RGBC) {
        return;
    }
    taskENTER_CRITICAL(&spinlock_);
    latest_sample_ = sample;
    has_sample_ = true;
    taskEXIT_CRITICAL(&spinlock_);
}

bool RgbcSampleStore::latest(model::NavigationSample& sample) const
{
    taskENTER_CRITICAL(&spinlock_);
    const bool available = has_sample_;
    if (available) {
        sample = latest_sample_;
    }
    taskEXIT_CRITICAL(&spinlock_);
    return available;
}

}  // namespace minicar::service

use core::task::Waker;
pub(crate) const QUEUE_CAPACITY: usize = 16;
struct Entry {
    at: u64,
    waker: Waker,
}
pub(crate) struct Queue {
    entries: [Option<Entry>; QUEUE_CAPACITY],
}
impl Queue {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [const { None }; QUEUE_CAPACITY],
        }
    }
    // Cloning and dropping Wakers belongs to the caller, outside its critical section.
    // Later deadlines for the same task retain the earliest outstanding wake.
    pub(crate) fn insert(&mut self, at: u64, waker: Waker) -> (Option<Waker>, Option<Waker>) {
        if let Some(entry) = self
            .entries
            .iter_mut()
            .flatten()
            .find(|e| e.waker.will_wake(&waker))
        {
            entry.at = entry.at.min(at);
            return (Some(waker), None);
        }
        let index = self
            .entries
            .iter()
            .position(Option::is_none)
            .unwrap_or_else(|| {
                self.entries
                    .iter()
                    .enumerate()
                    .max_by_key(|(_, e)| e.as_ref().unwrap().at)
                    .unwrap()
                    .0
            });
        let evicted = self.entries[index]
            .replace(Entry { at, waker })
            .map(|e| e.waker);
        (None, evicted)
    }
    pub(crate) fn drain(&mut self, now: u64, all: bool) -> [Option<Waker>; QUEUE_CAPACITY] {
        core::array::from_fn(|i| {
            if self.entries[i].as_ref().is_some_and(|e| all || e.at <= now) {
                self.entries[i].take().map(|e| e.waker)
            } else {
                None
            }
        })
    }
    pub(crate) fn next(&self) -> Option<u64> {
        self.entries.iter().flatten().map(|e| e.at).min()
    }
    pub(crate) fn len(&self) -> usize {
        self.entries.iter().flatten().count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        sync::{
            Arc,
            atomic::{AtomicUsize, Ordering},
        },
        task::Wake,
    };
    struct Count(AtomicUsize);
    impl Wake for Count {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }
    fn counter() -> (Arc<Count>, Waker) {
        let c = Arc::new(Count(AtomicUsize::new(0)));
        (c.clone(), Waker::from(c))
    }
    fn wake_all(batch: [Option<Waker>; QUEUE_CAPACITY]) {
        for w in batch.into_iter().flatten() {
            w.wake();
        }
    }
    #[test]
    fn same_task_keeps_earliest_and_callbacks_run_only_after_dequeue() {
        let mut q = Queue::new();
        let (c, w) = counter();
        drop(q.insert(50, w.clone()));
        drop(q.insert(100, w.clone()));
        assert_eq!(q.next(), Some(50));
        drop(q.insert(20, w.clone()));
        assert_eq!(q.len(), 1);
        assert_eq!(q.next(), Some(20));
        wake_all(q.drain(19, false));
        assert_eq!(c.0.load(Ordering::SeqCst), 0);
        let batch = q.drain(20, false);
        assert_eq!(q.len(), 0);
        assert_eq!(c.0.load(Ordering::SeqCst), 0);
        wake_all(batch);
        assert_eq!(c.0.load(Ordering::SeqCst), 1);
    }
    #[test]
    fn overflow_wakes_evicted_task_and_retains_new_deadline() {
        let mut q = Queue::new();
        let mut counters = std::vec::Vec::new();
        for i in 0..=QUEUE_CAPACITY {
            let (c, w) = counter();
            counters.push(c);
            let (old, wake) = q.insert(i as u64 + 1, w);
            drop(old);
            if let Some(w) = wake {
                w.wake();
            }
        }
        assert_eq!(q.len(), QUEUE_CAPACITY);
        assert_eq!(q.next(), Some(1));
        assert_eq!(counters[QUEUE_CAPACITY - 1].0.load(Ordering::SeqCst), 1);
        wake_all(q.drain(u64::MAX, false));
        assert!(counters.iter().all(|c| c.0.load(Ordering::SeqCst) == 1));
    }
    #[test]
    fn concurrent_change_drains_all_for_repoll_without_losing_far_future() {
        let mut q = Queue::new();
        let (c, w) = counter();
        drop(q.insert(u64::MAX, w));
        assert_eq!(q.next(), Some(u64::MAX));
        assert_eq!(q.len(), 1);
        wake_all(q.drain(0, true));
        assert_eq!(c.0.load(Ordering::SeqCst), 1);
        assert_eq!(q.next(), None);
    }
}

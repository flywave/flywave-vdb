package vdb

import (
	"errors"
	"sync"
)

type OnMissHandler func(string) (Cacheable, error)

type Cache struct {
	lock    sync.RWMutex
	size    int64
	maxSize int64
	entries map[string]*cacheEntry
	mostRU  *cacheEntry
	leastRU *cacheEntry
	onMiss  OnMissHandler
}

type Cacheable interface{}

type SizeAware interface {
	Size() int64
}

func getSize(x Cacheable) int64 {
	if s, ok := x.(SizeAware); ok {
		return s.Size()
	}
	return 1
}

type PurgeReason int

const (
	CACHEFULL PurgeReason = iota
	EXPLICITDELETE
	KEYCOLLISION
)

type NotifyPurge interface {
	OnPurge(why PurgeReason)
}

type cacheEntry struct {
	payload Cacheable
	id      string
	older   *cacheEntry
	younger *cacheEntry
}

func safeOnPurge(c Cacheable, why PurgeReason) {
	if t, ok := c.(NotifyPurge); ok {
		t.OnPurge(why)
	}
	return
}

func removeEntry(c *Cache, e *cacheEntry) {
	delete(c.entries, e.id)
	if e.older == nil {
		c.leastRU = e.younger
	} else {
		e.older.younger = e.younger
	}
	if e.younger == nil {
		c.mostRU = e.older
	} else {
		e.younger.older = e.older
	}
	c.size -= getSize(e.payload)
	return
}

func purgeLRU(c *Cache) {
	safeOnPurge(c.leastRU.payload, CACHEFULL)
	removeEntry(c, c.leastRU)
	return
}

func trimCache(c *Cache) {
	if c.maxSize <= 0 {
		return
	}
	for c.size > c.maxSize {
		purgeLRU(c)
	}
	return
}

func directSet(c *Cache, id string, payload Cacheable) {
	if old, ok := c.entries[id]; ok {
		safeOnPurge(old.payload, KEYCOLLISION)
		removeEntry(c, old)
	}
	e := cacheEntry{payload: payload, id: id}
	c.entries[id] = &e
	size := getSize(payload)
	if size == 0 {
		return
	}
	if c.leastRU == nil {
		c.leastRU = &e
		c.mostRU = &e
		e.younger = nil
		e.older = nil
	} else {
		c.mostRU.younger = &e
		e.older = c.mostRU
		c.mostRU = &e
	}
	c.size += size
	trimCache(c)
	return
}

func handleCacheMiss(c *Cache, id string) (Cacheable, error) {
	var val Cacheable
	var err error = ErrNotFound
	c.lock.RLock()
	onmiss := c.onMiss
	c.lock.RUnlock()
	if onmiss != nil {
		val, err = onmiss(id)
		if err == nil {
			if val != nil {
				c.lock.Lock()
				defer c.lock.Unlock()
				directSet(c, id, val)
			} else {
				err = ErrNotFound
			}
		}
	}
	return val, err
}

func (c *Cache) Init(maxsize int64) {
	c.maxSize = maxsize
	c.entries = map[string]*cacheEntry{}
	return
}

func (c *Cache) Set(id string, p Cacheable) {
	if p == nil {
		panic("Cacheable value must not be nil")
	}
	c.lock.Lock()
	defer c.lock.Unlock()
	directSet(c, id, p)
}

var ErrNotFound = errors.New("Key not found in cache")

func (c *Cache) Get(id string) (Cacheable, error) {
	c.lock.Lock()
	e, ok := c.entries[id]
	if !ok {
		c.lock.Unlock()
		return handleCacheMiss(c, id)
	}
	defer c.lock.Unlock()

	if e.younger == nil {
		return e.payload, nil
	}
	if e.older != nil {
		e.older.younger = e.younger
	} else {
		c.leastRU = e.younger
	}
	e.younger.older = e.older
	e.older = c.mostRU
	c.mostRU = e
	e.younger = nil
	e.older.younger = e

	return e.payload, nil
}

func (c *Cache) Delete(id string) {
	c.lock.Lock()
	defer c.lock.Unlock()

	e, ok := c.entries[id]
	if ok {
		safeOnPurge(e.payload, EXPLICITDELETE)
		if getSize(e.payload) != 0 {
			removeEntry(c, e)
		}
	}
	return
}

func (c *Cache) OnMiss(f OnMissHandler) {
	c.lock.Lock()
	defer c.lock.Unlock()
	c.onMiss = f
}

func (c *Cache) MaxSize(i int64) {
	c.lock.Lock()
	defer c.lock.Unlock()
	c.maxSize = i
	trimCache(c)
}

func (c *Cache) Size() int64 {
	c.lock.RLock()
	defer c.lock.RUnlock()
	return c.size
}

func (c *Cache) Close() error {
	return nil
}

func (c *Cache) Collect() error {
	c.lock.Lock()
	defer c.lock.RUnlock()
	for id := range c.entries {
		e, ok := c.entries[id]
		if ok {
			safeOnPurge(e.payload, EXPLICITDELETE)
			if getSize(e.payload) != 0 {
				removeEntry(c, e)
			}
		}
	}
	c.size = 0
	c.entries = map[string]*cacheEntry{}
	return nil
}

func NewCache(maxsize int64) *Cache {
	var mem Cache
	c := &mem
	c.Init(maxsize)
	return c
}

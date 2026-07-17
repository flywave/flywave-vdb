package vdb

import (
	"errors"
	"testing"
)

func TestCacheInit(t *testing.T) {
	c := &Cache{}
	c.Init(1000)
	if c == nil {
		t.Fatal("Cache nil after Init")
	}
}

func TestCacheGetNoHandler(t *testing.T) {
	c := &Cache{}
	c.Init(100)
	_, err := c.Get("missing")
	if err == nil {
		t.Errorf("Get(missing) should error without miss handler")
	}
}

func TestCacheSetGet(t *testing.T) {
	c := &Cache{}
	c.Init(1000)
	c.Set("key1", "value1")
	val, err := c.Get("key1")
	if err != nil {
		t.Fatalf("Get failed: %v", err)
	}
	if val.(string) != "value1" {
		t.Errorf("Get = %v, want value1", val)
	}
}

func TestCacheDelete(t *testing.T) {
	c := &Cache{}
	c.Init(1000)
	c.Set("k", "v")
	if _, err := c.Get("k"); err != nil {
		t.Fatalf("Get before delete failed: %v", err)
	}
	c.Delete("k")
	_, err := c.Get("k")
	if err == nil {
		t.Errorf("Get after Delete should error")
	}
}

func TestCacheOnMiss(t *testing.T) {
	c := &Cache{}
	c.Init(1000)
	missCalled := false
	c.OnMiss(func(id string) (Cacheable, error) {
		missCalled = true
		if id != "new-key" {
			t.Errorf("miss handler got id=%q, want new-key", id)
		}
		return "generated", nil
	})
	val, err := c.Get("new-key")
	if err != nil {
		t.Fatalf("Get via miss handler failed: %v", err)
	}
	if !missCalled {
		t.Errorf("miss handler was not called")
	}
	if val.(string) != "generated" {
		t.Errorf("Get = %v, want generated", val)
	}
}

func TestCacheOnMissError(t *testing.T) {
	c := &Cache{}
	c.Init(100)
	myErr := errors.New("miss error")
	c.OnMiss(func(id string) (Cacheable, error) {
		return nil, myErr
	})
	_, err := c.Get("err-key")
	if err != myErr {
		t.Errorf("expected miss handler error, got %v", err)
	}
}

func TestCacheSize(t *testing.T) {
	c := &Cache{}
	c.Init(100)
	if c.Size() != 0 {
		t.Errorf("initial Size = %d, want 0", c.Size())
	}
	c.Set("a", "1")
	if c.Size() <= 0 {
		t.Errorf("Size after Set should be > 0")
	}
}

func TestCacheKeySizeEviction(t *testing.T) {
	c := &Cache{}
	c.Init(50)
	payload := &sizeAwareTestPayload{data: make([]byte, 30), name: "p1"}
	c.Set("k1", payload)
	val, err := c.Get("k1")
	if err != nil {
		t.Errorf("Get k1 should succeed: %v", err)
	}
	if val.(*sizeAwareTestPayload).name != "p1" {
		t.Errorf("wrong value")
	}
}

type sizeAwareTestPayload struct {
	data []byte
	name string
}

func (p *sizeAwareTestPayload) Size() int64 {
	return int64(len(p.data))
}

func TestCacheSetOverwrite(t *testing.T) {
	c := &Cache{}
	c.Init(100)
	c.Set("k", "v1")
	c.Set("k", "v2")
	val, err := c.Get("k")
	if err != nil {
		t.Fatalf("Get after overwrite failed: %v", err)
	}
	if val.(string) != "v2" {
		t.Errorf("Get = %v, want v2", val)
	}
}

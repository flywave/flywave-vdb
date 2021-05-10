package vdb

type QuadtreePath struct {
	path uint64
}

const (
	QT_LEVEL_BITS = uint32(2)
	QT_LEVEL_BIT_MASK = uint64(0x03)
	QT_TOTAL_BITS = uint32(64)
	QT_DEFAULT_MAX_LEVEL = uint32(24)
	QT_PATH_MASK = ~(~uint64(0) >> (QT_DEFAULT_MAX_LEVEL * QT_LEVEL_BITS))
	QT_LEVEL_MASK = ~QT_PATH_MASK
	WEB_GLOBE_MAX_ZOOM =  uint32(20)
)

const (
	_order = [][2]uint64{{0, 3}, {1, 2}}
)

func Min(x, y uint32) uint32 {
    if x < y {
        return x
    }
    return y
}

func NewQuadtreePath() *QuadtreePath {
	return &QuadtreePath{path: 0}
}

func NewQuadtreePathFromLevelAndRowCol(level uint32, row uint32, col uint32) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	for ( j := 0; j < int(level); ++j) {
	   right := int(0x01 & (col >> (level - j - 1)))
	   top := int(0x01 & (row >> (level - j - 1)))
	   p.path |= order[right][top] << (QT_TOTAL_BITS - ((j + 1) * QT_LEVEL_BITS));
	}
  
	p.path |= level;
	return p
}

func NewQuadtreePathFromLevelAndBitList(level uint32, blist []byte) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	p.from_branchlist(level, blist);
	return p
}

func NewQuadtreePathFromBitList(blist string) *QuadtreePath {
	p := &QuadtreePath{path: 0}
	p.from_branchlist(uint32(len(blist)),  []byte(blist));
	return p
}

func NewQuadtreePathFromBitList(other *QuadtreePath, level uint32) *QuadtreePath {
	lev := Min(level, other.GetLevel())
	p := &QuadtreePath{path: 0}
	p.path = other.path_bits(lev) | lev;
	return p
}

func geodeticToTile(lon float64,  lat float64, zoom uint8) *QuadtreePath {
	 x := Math.Floor((lon + 180.0) / 360.0 * Math.Pow(2.0, zoom));
	 y := Math.Floor((1.0 + Math.Log(Math.Tan(lat * Math.Pi / 180.0) +
									 1.0 / Math.cos(lat * Math.Pi / 180.0)) /
									 Math.Pi) /
					 2.0 * Math.Pow(2.0, zoom));
	return NewQuadtreePathFromLevelAndRowCol(zoom, y, x);
  }

  func getTileFromBox(minx float64, miny float64, maxx float64, maxy float64)  *QuadtreePath{
	x := [2]float64{minx, miny};
	y := [2]float64{maxx, maxy};
	lonlat2merc(x, y, 2, D2R);
	geometry::box<geometry::point<T, 2>> _box(geometry::point<T, 2>(x[0], y[0]),
											  geometry::point<T, 2>(x[1], y[1]));
	 _zoom := uint8(
		Math.Floor(Math.Log((2 * M_PI) / _box.size()) / Math.Log(2)) - 1)
  
	if (_box.size() == 0) {
	  _zoom = uint8(WEB_GLOBE_MAX_ZOOM)
	}

	if _zoom > uint8(WEB_GLOBE_MAX_ZOOM) {
	_zoom = WEB_GLOBE_MAX_ZOOM
    }
	return geodeticToTile((minx + maxx) / 2.0, (miny + maxy) / 2.0, _zoom)
  }

func NewQuadtreePathFromBox(box []float64) *QuadtreePath {
	p := getTileFromBox(box[0], box[1], box[2], box[3])
	return p
}

func (q *QuadtreePath) Valid() bool {
	return GetLevel() <= QT_DEFAULT_MAX_LEVEL &&
	(0 == (q.path & ~(q.path_mask(q.GetLevel()) | QT_LEVEL_MASK)))
}

func (q *QuadtreePath) Less(other *QuadtreePath) bool {
  minlev := (GetLevel() < other.GetLevel()) ? GetLevel() : other.GetLevel()
  mask := ~(~uint64(0) >> (minlev * QT_LEVEL_BITS))
  if (mask & (q.path ^ other.path)) {
    return path_bits() < other.path_bits()
  } else {
    return GetLevel() < other.GetLevel()
  }
}

func (q *QuadtreePath) Greater(other *QuadtreePath) bool {
	return other.Less(q)
}

func (q *QuadtreePath) Equal(other *QuadtreePath) bool {
	return other.path == q.path
}

func (q *QuadtreePath) Path() uint64 {
	return q.path
}

func (q *QuadtreePath) ToString() string {
	result := make([]byte, int(p.GetLevel()))
	
	for (i := 0; i < int(p.GetLevel()); ++i) {
	  result[i] = byte('0') + p.level_bits_at_pos(i)
	}
	return string(result)
}

func (q *QuadtreePath) GetGenerationSequence() uint64 {
	level_ := GetLevel()
	sequence := q.path
	 check_for_2_or_3_mask := (uint64(0x1)) << (QT_TOTAL_BITS - 1)
	 interchange_2_or_3_mask := (uint64(0x01)) << (QT_TOTAL_BITS - 2)
  
	for ( j := 0; j < int(level_)
		 ++j, check_for_2_or_3_mask >>= 2, interchange_2_or_3_mask >>= 2) {
	  if (sequence & check_for_2_or_3_mask) {
		sequence ^= interchange_2_or_3_mask
	  }
	}
	return sequence
}

func (q *QuadtreePath) Parent() *QuadtreePath {
	 new_level := p.GetLevel() - 1

	return &QuadtreePath{path:
		(p.path &
		 (QT_PATH_MASK << QT_LEVEL_BITS * (QT_DEFAULT_MAX_LEVEL - new_level))) |
		new_level};
}

func (q *QuadtreePath) Child(level uint32) *QuadtreePath {
	 new_level := p.GetLevel() + 1;
  return &QuadtreePath(path: p.path_bits() |
                       uint64(child)
                           << (QT_TOTAL_BITS - new_level * QT_LEVEL_BITS) |
                       new_level)
}

func (q *QuadtreePath) WhichChild() uint32 {
	return (q.path >> (QT_TOTAL_BITS - GetLevel() * QT_LEVEL_BITS)) &
	QT_LEVEL_BIT_MASK
}

func (q *QuadtreePath) AdvanceInLevel() bool {
	 path_bits_ := p.path_bits()
	 path_mask_ := p.path_mask(p.GetLevel())
	if (path_bits_ != path_mask_) {
	  p.path += uint64(1) << (QT_TOTAL_BITS - p.GetLevel() * QT_LEVEL_BITS);
	  return true
	} else {
	  return false
	}
}

func (q *QuadtreePath) Advance(max_level uint32) bool {
	if (p.GetLevel() < max_level) {
		q.path = p.Child(0).path
		return true
	  } else {
		for (p.WhichChild() == 4 - 1) {
			q.path = p.Parent()
		}
		return p.AdvanceInLevel()
	  }
}

func (q *QuadtreePath) IsAncestorOf(other *QuadtreePath) bool {
	if (p.GetLevel() <= other.GetLevel()) {
		return path_bits(p.GetLevel()) == other.path_bits(p.GetLevel())
	  } else {
		return false
	  }
}

func IsPostOrder(path1 *QuadtreePath, path2 *QuadtreePath) bool {
	return !path1.IsAncestorOf(path2) &&
           (path2.IsAncestorOf(path1) || path2.Greater(path1));
}

func (q *QuadtreePath) GetLevelRowCol() (level uint32, row uint32, col uint32) {
	rowbits := []uint32{0x00, 0x00, 0x01, 0x01}
	colbits := []uint32{0x00, 0x01, 0x01, 0x00}
  
	 row_val := uint32(0);
	 col_val := uint32(0);
  
	for ( j := 0; j < int(GetLevel()); ++j) {
	   level_bits := q.level_bits_at_pos(j);
	  row_val = (row_val << 1) | (rowbits[level_bits]);
	  col_val = (col_val << 1) | (colbits[level_bits]);
	}
  
	level_ = GetLevel();
	row = row_val;
	col = col_val;
	return
}

func (q *QuadtreePath) GetLevel() uint32 {
	return q.path & QT_LEVEL_MASK
}

func (q *QuadtreePath) ChildTileCoordinates(tile_width int32, child *QuadtreePath) (success bool, level uint32, row uint32, col uint32) {
	if (!q.IsAncestorOf(child)) {
		success = false
		return 
	  }
	
	   relative_qpath := RelativePath(q, child);
	   level = tile_width;
	  row = 0;
	  col = 0;
	  for (uint32_t level = 0; level < relative_qpath.level() && level > 1;
		   ++level) {
		 quad := relative_qpath.Get(level)
		level >>= 1;
		if (quad == 0) {
			row += level;
		} else if (quad == 1) {
			row += level;
			col += level;
		} else if (quad == 2) {
			col += level;
		}
	  }
	  success = true
	  return 
}

func (q *QuadtreePath) Concatenate(sub_path *QuadtreePath) *QuadtreePath {
	 level_ = GetLevel() + sub_path.GetLevel();
	return &QuadtreePath{path:
		(path_ & QT_PATH_MASK) |
		((sub_path.path_ & QT_PATH_MASK) >> GetLevel() * QT_LEVEL_BITS) | level_};
}

func (q *QuadtreePath) ToIndex(level uint32) uint64 {
    return (q.path >> (QT_TOTAL_BITS - level * QT_LEVEL_BITS))
}

func (q *QuadtreePath) Get(position uint32) uint32 {
	return q.level_bits_at_pos(position)
}

func (q *QuadtreePath) path_bits() uint64 { return q.path & QT_PATH_MASK }

func (q *QuadtreePath) path_mask(level uint32) uint64 {
  return QT_PATH_MASK << ((QT_DEFAULT_MAX_LEVEL - level) * QT_LEVEL_BITS)
}

func (q *QuadtreePath) path_bits_level(level uint32) uint64 {
  return q.path & path_mask(level)
}

func (q *QuadtreePath) level_bits_at_pos(position uint32) uint32 {
  return (q.path >> (QT_TOTAL_BITS - (position + 1) * QT_LEVEL_BITS)) &
  QT_LEVEL_BIT_MASK
}

func (q *QuadtreePath) from_branchlist(level uint32, blist []byte) {
for ( j := 0; j < int(level); ++j) {
	q.path |= (blist[j] & QT_LEVEL_BIT_MASK)
<< (QT_TOTAL_BITS - ((j + 1) * QT_LEVEL_BITS));
}
q.path |= level;
}

func RelativePath(parent *QuadtreePath, child *QuadtreePath) *QuadtreePath {
	levelDiff := child.GetLevel() - parent.GetLevel()
	return &QuadtreePath{path: (child.path_bits() << (parent.GetLevel() * QT_LEVEL_BITS)) |
						 levelDiff};
}

func quadToBufferOffset(quad uint32,  tileWidth uint32,
	 tileHeight uint32) uint32 {
		switch quad {
		case 0:
		  return 0;
		case 1:
		  return tileWidth / 2;
		case 2:
		  return (tileHeight * tileWidth) / 2;
		case 3:
		  return ((tileHeight + 1) * tileWidth) / 2;
		}
		return 0;
}

func  magnifyQuadAddr( inRow uint32,  inCol uint32,
	 inQuad uint32) (outRow uint32,
		outCol uint32) {
			switch inQuad {
			case 0:
			  outRow = inRow * 2;
			  outCol = inCol * 2;
			  break;
			case 1:
			  outRow = inRow * 2;
			  outCol = (inCol * 2) + 1;
			  break;
			case 2:
			  outRow = (inRow * 2) + 1;
			  outCol = inCol * 2;
			  break;
			case 3:
			  outRow = (inRow * 2) + 1;
			  outCol = (inCol * 2) + 1;
			  break;
			}
			return
		}
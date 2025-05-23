package librealsense2

/*
#cgo linux darwin LDFLAGS: -L/usr/local/lib/ -lrealsense2
#cgo CPPFLAGS: -I/usr/local/include
#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/h/rs_processing.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_context.h>

// 定义RealSense常量
#define RS2_API_VERSION RS2_API_MAJOR_VERSION * 10000 + RS2_API_MINOR_VERSION * 100 + RS2_API_PATCH_VERSION

// 定义枚举类型
typedef enum rs2_stream          rs2_stream;
typedef enum rs2_format          rs2_format;
typedef enum rs2_extension       rs2_extension;
typedef enum rs2_camera_info     rs2_camera_info;

// 定义结构体类型
typedef struct rs2_context           rs2_context;
typedef struct rs2_device            rs2_device;
typedef struct rs2_device_list        rs2_device_list;
typedef struct rs2_pipeline          rs2_pipeline;
typedef struct rs2_config            rs2_config;
typedef struct rs2_pipeline_profile  rs2_pipeline_profile;
typedef struct rs2_frame             rs2_frame;
typedef struct rs2_processing_block  rs2_processing_block;
typedef struct rs2_error             rs2_error;

// 定义函数原型
rs2_context* rs2_create_context(int api_version, rs2_error** error);
void rs2_delete_context(rs2_context* context);
rs2_device_list* rs2_query_devices(rs2_context* context, rs2_error** error);
int rs2_get_device_count(const rs2_device_list* info_list, rs2_error** error);
rs2_device* rs2_create_device(const rs2_device_list* info_list, int index, rs2_error** error);
void rs2_delete_device(rs2_device* device);
const char* rs2_get_device_info(const rs2_device* device, rs2_camera_info info, rs2_error** error);
void rs2_delete_device_list(rs2_device_list* info_list);
rs2_pipeline* rs2_create_pipeline(rs2_context* ctx, rs2_error** error);
void rs2_delete_pipeline(rs2_pipeline* pipeline);
rs2_config* rs2_create_config(rs2_error** error);
void rs2_delete_config(rs2_config* config);
void rs2_config_enable_device(rs2_config* config, const char* serial, rs2_error** error);
void rs2_config_enable_stream(rs2_config* config, rs2_stream stream, int index, int width, int height, rs2_format format, int framerate, rs2_error** error);
rs2_pipeline_profile* rs2_pipeline_start_with_config(rs2_pipeline* pipeline, rs2_config* config, rs2_error** error);
void rs2_pipeline_stop(rs2_pipeline* pipeline, rs2_error** error);
void rs2_delete_pipeline_profile(rs2_pipeline_profile* profile);
rs2_frame* rs2_pipeline_wait_for_frames(rs2_pipeline* pipeline, unsigned int timeout_ms, rs2_error** error);
int rs2_embedded_frames_count(const rs2_frame* composite, rs2_error** error);
rs2_frame* rs2_extract_frame(const rs2_frame* composite, int index, rs2_error** error);
void rs2_release_frame(rs2_frame* frame);
int rs2_get_frame_width(const rs2_frame* frame, rs2_error** error);
int rs2_get_frame_height(const rs2_frame* frame, rs2_error** error);
const void* rs2_get_frame_data(const rs2_frame* frame, rs2_error** error);
int rs2_is_frame_extendable_to(const rs2_frame* frame, rs2_extension extension, rs2_error** error);
int rs2_get_frame_bytes_per_pixel(const rs2_frame* frame, rs2_error** error);
rs2_processing_block* rs2_create_align(rs2_stream align_to, rs2_error** error);
void rs2_delete_processing_block(rs2_processing_block* block);
rs2_frame* rs2_process_frame(rs2_processing_block* block, rs2_frame* frame, rs2_error** error);
rs2_frame* rs2_import_frame_from_raw_data(const void* data, int width, int height, int stride, rs2_format format, int stride_in_pixels, int bpp, rs2_error** error);
const char* rs2_get_error_message(const rs2_error* error);
void rs2_free_error(rs2_error* error);
*/
import "C"
import (
	"errors"
	"fmt"
	"time"
	"unsafe"

	"gocv.io/x/gocv"
)

const defaultTimeout = time.Second

var (
	ErrNotDevicesFound = errors.New("no realsense devices found")
)

// Stream is a RealSense data stream.
type Stream C.rs2_stream

const (
	StreamDepth Stream = C.RS2_STREAM_DEPTH
	StreamColor Stream = C.RS2_STREAM_COLOR
)

// Pipepline represents a realsense pipeline, includes a device and a selection
// of active streams.
type Pipeline struct {
	p       *C.rs2_pipeline
	ctx     *C.rs2_context
	conf    *C.rs2_config
	profile *C.rs2_pipeline_profile

	filters []*Filter
}

// 在librealsense2包中添加以下代码
const (
	// Stream types
	StreamDepth Stream = C.RS2_STREAM_DEPTH
	StreamColor Stream = C.RS2_STREAM_COLOR

	// Format types
	FormatZ16  Format = C.RS2_FORMAT_Z16
	FormatBGR8 Format = C.RS2_FORMAT_BGR8

	// Camera info types
	CameraInfoSerialNumber CameraInfo = C.RS2_CAMERA_INFO_SERIAL_NUMBER

	// Extension types
	ExtensionDepthFrame Extension = C.RS2_EXTENSION_DEPTH_FRAME
)

type Stream C.rs2_stream
type Format C.rs2_format
type CameraInfo C.rs2_camera_info
type Extension C.rs2_extension

// Align represents a frame alignment processor
type Align struct {
	align *C.rs2_processing_block
}

// NewAlign creates a new frame aligner that aligns frames to the specified target stream
func NewAlign(stream Stream) (*Align, error) {
	var err *C.rs2_error
	align := C.rs2_create_align(C.rs2_stream(stream), &err)
	if err != nil {
		return nil, errorFrom(err)
	}
	return &Align{align: align}, nil
}

// Process aligns the input frame to the target stream
func (a *Align) Process(frame *gocv.Mat) (*gocv.Mat, error) {
	if frame == nil || frame.Empty() {
		return nil, fmt.Errorf("input frame is nil or empty")
	}

	var err *C.rs2_error
	
	// 将gocv.Mat转换为rs2_frame
	var frameType C.rs2_format
	switch frame.Type() {
	case gocv.MatTypeCV8UC3:
		frameType = C.RS2_FORMAT_BGR8
	case gocv.MatTypeCV16UC1:
		frameType = C.RS2_FORMAT_Z16
	default:
		return nil, fmt.Errorf("unsupported frame format")
	}
	
	w := frame.Cols()
	h := frame.Rows()
	
	// 创建rs2_frame
	frameData := frame.ToBytes()
	rs2Frame := C.rs2_import_frame_from_raw_data(
		unsafe.Pointer(&frameData[0]),
		C.int(w),
		C.int(h),
		C.int(frame.Step()),
		frameType,
		C.int(0), // stride
		C.int(0), // bits per pixel
		&err)
	if err != nil {
		return nil, errorFrom(err)
	}
	defer C.rs2_release_frame(rs2Frame)
	
	// 处理对齐
	processedFrames := C.rs2_process_frame(a.align, rs2Frame, &err)
	if err != nil {
		return nil, errorFrom(err)
	}
	defer C.rs2_release_frame(processedFrames)
	
	// 提取对齐后的帧
	count := C.rs2_embedded_frames_count(processedFrames, &err)
	if err != nil {
		return nil, errorFrom(err)
	}
	
	if count == 0 {
		return nil, fmt.Errorf("no frames after processing")
	}
	
	alignedFrame := C.rs2_extract_frame(processedFrames, 0, &err)
	if err != nil {
		return nil, errorFrom(err)
	}
	defer C.rs2_release_frame(alignedFrame)
	
	// 将rs2_frame转换回gocv.Mat
	w = int(C.rs2_get_frame_width(alignedFrame, &err))
	h = int(C.rs2_get_frame_height(alignedFrame, &err))
	
	var matType gocv.MatType
	if C.rs2_is_frame_extendable_to(alignedFrame, C.RS2_EXTENSION_DEPTH_FRAME, &err) != 0 {
		matType = gocv.MatTypeCV16UC1
	} else {
		matType = gocv.MatTypeCV8UC3
	}
	
	frameDataPtr := C.rs2_get_frame_data(alignedFrame, &err)
	if err != nil {
		return nil, errorFrom(err)
	}
	
	dataSize := w * h * int(C.rs2_get_frame_bytes_per_pixel(alignedFrame, &err))
	if err != nil {
		return nil, errorFrom(err)
	}
	
	data := C.GoBytes(unsafe.Pointer(frameDataPtr), C.int(dataSize))
	result, err := gocv.NewMatFromBytes(h, w, matType, data)
	if err != nil {
		return nil, err
	}
	
	return &result, nil
}

// Close releases the align resources
func (a *Align) Close() error {
	if a.align != nil {
		C.rs2_delete_processing_block(a.align)
		a.align = nil
	}
	return nil
}

// NewPipeline creates a new pipeline with the given serial number. If serial is
// empty, the first device will be used.
func NewPipeline(serial string) (*Pipeline, error) {
	var err *C.rs2_error
	ctx := C.rs2_create_context(C.RS2_API_VERSION, &err)
	if err != nil {
		return nil, errorFrom(err)
	}

	device_list := C.rs2_query_devices(ctx, &err)
	if err != nil {
		return nil, errorFrom(err)
	}

	dev_count := C.rs2_get_device_count(device_list, &err)
	if err != nil {
		return nil, errorFrom(err)
	}

	if C.int(dev_count) == 0 {
		return nil, ErrNotDevicesFound
	}

	conf := C.rs2_create_config(&err)
	if err != nil {
		return nil, errorFrom(err)
	}

	if serial != "" {
		for i := 0; i < int(dev_count); i++ {
			dev := C.rs2_create_device(device_list, C.int(i), &err)
			if err != nil {
				fmt.Println(errorFrom(err))
				continue
			}

			s := C.rs2_get_device_info(dev, C.RS2_CAMERA_INFO_SERIAL_NUMBER, &err)
			if C.GoString(s) == serial {
				if C.rs2_config_enable_device(conf, s, &err); err != nil {
					return nil, errorFrom(err)
				}
			}

			C.rs2_delete_device(dev)
		}
	}

	p := C.rs2_create_pipeline(ctx, &err)
	if err != nil {
		return nil, errorFrom(err)
	}

	C.rs2_delete_device_list(device_list)

	pl := &Pipeline{
		p:    p,
		ctx:  ctx,
		conf: conf,
	}

	return pl, nil
}

// EnableStream enables a stream with the given width, height and fps.
func (p *Pipeline) EnableStream(stream Stream, width, height, fps int) error {
	var format C.rs2_format
	switch stream {
	case StreamDepth:
		format = C.RS2_FORMAT_Z16
	case StreamColor:
		format = C.RS2_FORMAT_BGR8
	default:
		return fmt.Errorf("unknown stream %d", stream)
	}

	var err *C.rs2_error
	C.rs2_config_enable_stream(p.conf, C.rs2_stream(stream), 0, C.int(width), C.int(height), format, C.int(fps), &err)

	if err != nil {
		return errorFrom(err)
	}

	return nil
}

// SetDefaultPostProcessingFilters sets the default post-processing filters.
func (p *Pipeline) SetDefaultPostProcessingFilters() {
	p.SetPostProcessingFilters(
		DefaultDecimationFilter(),
		DepthDisparityFilter(),
		DefaultSpatialFilter(),
		DefaultTemporalFilter(),
		DefaultHoleFillingFilter(),
		DisparityToDepthFilter(),
	)
}

// SetPostProcessingFilters sets the post-processing filters to be applied to the
// frames. The post-processing filters are designed and built for
// concatenation into processing pipes. There are no software-imposed constraints
// that mandate the order in which the filters shall be applied. At the same
// time the recommended scheme used in librealsense tools and demos is elaborated
// below:
//
// Depth Frame >> Decimation Filter >> Depth2Disparity Transform** >>
// Spatial Filter >> Temporal Filter >> Disparity2Depth Transform** >>
// Hole Filling Filter >> Filtered Depth.
func (p *Pipeline) SetPostProcessingFilters(f ...*Filter) {
	p.filters = f
}

// Start starts the pipeline.
func (p *Pipeline) Start() error {
	for _, f := range p.filters {
		f.initialize()
	}

	var err *C.rs2_error
	prof := C.rs2_pipeline_start_with_config(p.p, p.conf, &err)
	if err != nil {
		return errorFrom(err)
	}

	p.profile = prof
	return nil
}

// Close closes the pipeline and frees all resources.
func (p *Pipeline) Close() error {
	var err *C.rs2_error
	C.rs2_pipeline_stop(p.p, &err)
	if err != nil {
		return errorFrom(err)
	}

	C.rs2_delete_pipeline_profile(p.profile)
	C.rs2_delete_config(p.conf)
	C.rs2_delete_pipeline(p.p)
	C.rs2_delete_context(p.ctx)

	var errs []error
	for _, f := range p.filters {
		if err := f.Close(); err != nil {
			errs = append(errs, err)
		}
	}

	return errors.Join(errs...)
}

// WaitFrames waits for frames from the pipeline. Color and depth frames are
// sent to the frame channel. If an error occurs, it is sent to the error channel.
// If the error channel is nil, the error is ignored.
func (p *Pipeline) WaitFrames(frameCh chan *gocv.Mat, errCh chan error, timeout time.Duration) error {
	if timeout == 0 {
		timeout = defaultTimeout
	}

	ms := timeout.Milliseconds()

	for {
		var errc *C.rs2_error
		frames := C.rs2_pipeline_wait_for_frames(p.p, C.uint(ms), &errc)
		if errc != nil {
			if errCh != nil {
				errCh <- errorFrom(errc)
			}

			continue
		}

		count := C.rs2_embedded_frames_count(frames, &errc)
		if errc != nil {
			errCh <- errorFrom(errc)
			continue
		}

		for i := 0; i < int(count); i++ {
			var ret gocv.Mat
			var err error

			frame := C.rs2_extract_frame(frames, C.int(i), &errc)
			if frame == nil {
				continue
			}

			if C.rs2_is_frame_extendable_to(frame, C.RS2_EXTENSION_DEPTH_FRAME, &errc) != 0 {
				for _, f := range p.filters {
					filtered, err := f.Apply(frame, 0)
					if err != nil {
						if errCh != nil {
							errCh <- errorFrom(errc)
						}
					}

					frame = filtered
				}
			}

			w := C.rs2_get_frame_width(frame, &errc)
			h := C.rs2_get_frame_height(frame, &errc)

			t := gocv.MatTypeCV8UC3
			sz := w * h * 3
			if C.rs2_is_frame_extendable_to(frame, C.RS2_EXTENSION_DEPTH_FRAME, &errc) != 0 {
				t = gocv.MatTypeCV16UC1
				sz = w * h * 2
			}

			frameData := C.rs2_get_frame_data(frame, &errc)
			if errc != nil {
				if errCh != nil {
					errCh <- errorFrom(errc)
				}

				continue
			}

			b := C.GoBytes(unsafe.Pointer(frameData), C.int(sz))
			ret, err = gocv.NewMatFromBytes(int(h), int(w), t, b)
			if err != nil {
				if errCh != nil {
					errCh <- err
				}
			} else {
				frameCh <- &ret
			}

			C.rs2_release_frame(frame)
		}

		C.rs2_release_frame(frames)
	}
}

func errorFrom(err *C.rs2_error) error {
	if err == nil {
		return nil
	}

	defer C.rs2_free_error(err)
	return fmt.Errorf(C.GoString(C.rs2_get_error_message(err)))
}

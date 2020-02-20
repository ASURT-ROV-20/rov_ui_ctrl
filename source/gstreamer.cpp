#include "include/gstreamer.h"
#include <QtWidgets/QWidget>

Gstreamer::Gstreamer(Ui::MainWindow * ui)
{
    this->ui = ui;
    initGstreamer();
}


void Gstreamer::initGstreamer() {
    gst_init(nullptr, nullptr);
    initPipeline(&gst_pipeline1, &gst_sink1,
                 "rtspsrc location='rtsp://admin:123456@10.42.0.13/media/video1' latency=0 drop-latency=true max-rtcp-rtp-time-diff=50 protocols=udp-mcast+udp ! rtph265depay ! avdec_h265 ! ximagesink  name=mySink",
                 "mySink");
    createWindow(gst_sink1, gst_pipeline1, ui->camera1Wdgt);

    initPipeline(&gst_pipeline2, &gst_sink2,
                 "rtspsrc location='rtsp://admin:123456@10.42.0.13/media/video1' latency=0 drop-latency=true max-rtcp-rtp-time-diff=50 protocols=udp-mcast+udp ! rtph265depay ! avdec_h265 ! ximagesink  name=mySink2",
                 "mySink2");
    createWindow(gst_sink2, gst_pipeline2, ui->camera2Wdgt);

    initPipeline(&gst_pipeline3, &gst_sink3,
                 "rtspsrc location='rtsp://admin:123456@10.42.0.13/media/video1' latency=0 drop-latency=true max-rtcp-rtp-time-diff=50 protocols=udp-mcast+udp ! rtph265depay ! avdec_h265 ! ximagesink  name=mySink3",
                 "mySink3");
    createWindow(gst_sink3, gst_pipeline3, ui->camera3Wdgt);
}

void Gstreamer::initPipeline(GstElement ** gst_pipeline, GstElement ** gst_sink, char const * pipeline_string, char const * name) {
    *gst_pipeline = gst_parse_launch(pipeline_string, nullptr);
    *gst_sink = gst_bin_get_by_name((GstBin*)(*gst_pipeline), name);
}

WId Gstreamer::createWindow(GstElement * gst_sink, GstElement * gst_pipeline, QWidget * cameraWidget) {
    WId xwinid = cameraWidget->winId();
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (gst_sink), static_cast<guintptr>(xwinid));
    gst_element_set_state (gst_pipeline, GST_STATE_PLAYING);
    return xwinid;
}

void Gstreamer::closeGstreamer() {
    gst_object_unref(gst_pipeline1);
    gst_object_unref(gst_pipeline2);
    gst_object_unref(gst_pipeline3);
}

Gstreamer::~Gstreamer()
{
    closeGstreamer();
}

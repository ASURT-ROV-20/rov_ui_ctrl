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
                 "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink",
                 "mySink");
    createWindow(gst_sink1, gst_pipeline1, ui->camera1Wdgt);

    initPipeline(&gst_pipeline2, &gst_sink2,
                 "udpsrc port=5001 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink2",
                 "mySink2");
    createWindow(gst_sink2, gst_pipeline2, ui->camera2Wdgt);
}

void Gstreamer::initPipeline(GstElement ** gst_pipeline, GstElement ** gst_sink, char const * pipeline_string, char const * name) {
    *gst_pipeline = gst_parse_launch(pipeline_string, nullptr);
    *gst_sink = gst_bin_get_by_name((GstBin*)(*gst_pipeline), name);
}

WId Gstreamer::createWindow(GstElement * gst_sink, GstElement * gst_pipeline, QWidget * cameraWidget) {
    WId xwinid = cameraWidget->winId();
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (gst_sink), (guintptr)xwinid);
    gst_element_set_state (gst_pipeline, GST_STATE_PLAYING);
    return xwinid;
}

void Gstreamer::closeGstreamer() {
    gst_object_unref(gst_pipeline1);
    gst_object_unref(gst_pipeline2);
}

Gstreamer::~Gstreamer()
{
    closeGstreamer();
}

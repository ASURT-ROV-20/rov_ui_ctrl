#ifndef GSTREAMER_H
#define GSTREAMER_H

#include <glib-object.h>
#include <gst/gst.h>
#include <gst/video/video.h>
#include "ui_mainwindow.h"

class Gstreamer
{
public:
    Gstreamer(Ui::MainWindow * ui);
    ~Gstreamer();

    void initGstreamer();
    void closeGstreamer();

private:
    Ui::MainWindow * ui;
    GstElement * gst_pipeline1;
    GstElement * gst_sink1;
    GstElement * gst_pipeline2;
    GstElement * gst_sink2;
    GstElement * gst_pipeline3;
    GstElement * gst_sink3;

    void initPipeline(GstElement ** gst_pipeline, GstElement ** gst_sink, char const * pipeline_string, char const * name);
    WId createWindow(GstElement * gst_sink, GstElement * gst_pipeline, QWidget * cameraWidget);
};

#endif // GSTREAMER_H

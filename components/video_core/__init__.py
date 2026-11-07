"""Frame types, buffer pools and source/sink interfaces shared by the
esphome-video components. Loaded automatically by the components that use it."""

import esphome.codegen as cg

CODEOWNERS = ["@onetxpanda"]

video_ns = cg.esphome_ns.namespace("video")
ImageSource = video_ns.class_("ImageSource")
ImageSink = video_ns.class_("ImageSink")
RadiometricSource = video_ns.class_("RadiometricSource")
RadiometricSink = video_ns.class_("RadiometricSink")

import 'dart:ui';

import 'package:dorvan/application/constants/app_infos.dart';
import 'package:flutter/material.dart';

class ResponsiveImage extends StatefulWidget {
  final String image;

  ResponsiveImage({this.image = ''});
  @override
  _ResponsiveImageState createState() => _ResponsiveImageState();
}

class _ResponsiveImageState extends State<ResponsiveImage>
    with WidgetsBindingObserver {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance!.addObserver(this);
    _computeOrientation();
    image = widget.image.substring(0, widget.image.length - 4);
    node = widget.image.substring(widget.image.length - 4, widget.image.length);
    _findImage();
  }

  @override
  void dispose() {
    WidgetsBinding.instance!.removeObserver(this);
    super.dispose();
  }

  double width = 0;
  double height = 0;
  bool lastOrientation = false;
  bool orientation = false; // 0 = landscape, 1 = portrait
  String image = '';
  String node = '';
  String responsiveImage = '';

  @override
  void didChangeMetrics() {
    _computeOrientation();

    if (orientation != lastOrientation) {
      setState(() {
        lastOrientation = orientation;
        _findImage();
      });
    }
  }

  void _computeOrientation() {
    width = window.physicalSize.width;
    height = window.physicalSize.height;
    orientation = width > height;
  }

  void _findImage() {
    responsiveImage = image + (orientation ? '_landscape' : '_portrait') + node;
  }

  @override
  Widget build(BuildContext context) {
    print(image);
    return Image.asset(
      kImagePath + responsiveImage,
      fit: BoxFit.cover,
    );
  }
}

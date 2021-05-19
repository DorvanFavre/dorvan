import 'dart:math';

import 'package:dorvan/application/constants/theme.dart';
import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/application/providers/page_controller_provider.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

class BottomBar extends StatefulWidget {
  final PageController? pageController;
  final int? totalPage;
  final Brightness? brightness;

  BottomBar(
      {@required this.pageController,
      @required this.totalPage,
      @required this.brightness});

  @override
  _BottomBarState createState() => _BottomBarState();
}

class _BottomBarState extends State<BottomBar> {
  ValueNotifier<double> barWidthNotifier = ValueNotifier(0);

  @override
  void initState() {
    super.initState();

    // update bar according to the current page
    widget.pageController!.addListener(() {
      barWidthNotifier.value = (MediaQuery.of(context).size.width - 60) *
          widget.pageController!.page! /
          (widget.totalPage! - 1).toDouble();
    });
  }

  @override
  void dispose() {
    super.dispose();

    barWidthNotifier.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        // Chapter sliding bar
        Stack(
          children: [
            Container(
              width: MediaQuery.of(context).size.width - 60,
              height: 1,
              color: widget.brightness == Brightness.light
                  ? Colors.grey[300]
                  : Colors.white30,
            ),
            ValueListenableBuilder<double>(
                valueListenable: barWidthNotifier,
                builder: (context, width, child) {
                  return Container(
                    width: max(10, width),
                    height: 1,
                    color: widget.brightness == Brightness.light
                        ? kArticleDescriptionColor
                        : kHudLightColor,
                  );
                })
          ],
        ),
      ],
    );
  }
}

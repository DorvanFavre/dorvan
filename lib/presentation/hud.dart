import 'package:dorvan/application/constants/theme.dart';
import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/application/providers/page_controller_provider.dart';
import 'package:dorvan/presentation/bottom_bar.dart';
import 'package:dorvan/presentation/language_selection.dart';
import 'package:enum_to_string/enum_to_string.dart';
import 'package:flutter/material.dart';
import 'package:enum_to_string/enum_to_string.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'chapter_selection.dart';

class Hud extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: EdgeInsets.all(20),
      child: Stack(
        children: [
          // Menu
          Align(
              alignment: Alignment.topRight,
              child: IconButton(
                  onPressed: () {
                    showDialog(
                        barrierColor: Colors.black87,
                        context: context,
                        builder: (context) {
                          return Center(child: ChapterSelection());
                        });
                  },
                  icon: Icon(
                    Icons.menu_rounded,
                    color: kHudLightColor,
                  ))),

          // Language
          Align(
            alignment: Alignment.topLeft,
            child: IconButton(
              onPressed: () => showDialog(
                  barrierColor: Colors.black87,
                  context: context,
                  builder: (context) {
                    return Center(child: LanguageSelection());
                  }),
              icon: Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                mainAxisSize: MainAxisSize.min,
                children: [
                  Container(
                    height: 30,
                    width: 1,
                    color: kChapterDescriptionColor,
                  ),
                  SizedBox(width: 2),
                  Column(
                    mainAxisSize: MainAxisSize.min,
                    mainAxisAlignment: MainAxisAlignment.start,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Consumer(builder: (context, watch, child) {
                        final language = watch(languageStateProvider).state;
                        return Text(
                          EnumToString.convertToString(language),
                          style: TextStyle(fontSize: 14, color: kHudLightColor),
                        );
                      }),
                      /*SizedBox(height: 2),
                      Text(
                        '01',
                        style: TextStyle(
                            color: Colors.white54,
                            fontFamily: 'Futura',
                            fontSize: 12),
                      ),*/
                    ],
                  )
                ],
              ),
            ),
          ),
          Align(
              alignment: Alignment.bottomCenter,
              child: BottomBar(
                pageController: context.read(pageControllerProvider),
                totalPage: context.read(appContentProvider).length,
                brightness: Brightness.dark,
              ))
        ],
      ),
    );
  }
}

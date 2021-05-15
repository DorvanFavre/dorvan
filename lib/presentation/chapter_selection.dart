import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/application/providers/page_controller_provider.dart';
import 'package:dorvan/domain/entity/language.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/presentation/aware_text.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:riverpod/riverpod.dart';
import 'package:enum_to_string/enum_to_string.dart';

class ChapterSelection extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: context.read(appContentProvider).map((pageContent) {
          return ChapterTitle(
            pageContent: pageContent,
          );
        }).toList());
  }
}

class ChapterTitle extends StatefulWidget {
  final PageContent? pageContent;

  ChapterTitle({@required this.pageContent});
  @override
  _ChapterTitleState createState() => _ChapterTitleState();
}

class _ChapterTitleState extends State<ChapterTitle> {
  PageController? pageController;
  int? pageIndex;
  int? thisPageIndex;
  bool? enable;

  @override
  void initState() {
    super.initState();

    pageController = context.read(pageControllerProvider);
    //print('pagecontroller : ' + pageController.toString());

    thisPageIndex =
        context.read(appContentProvider).indexOf(widget.pageContent!);
    //print('thisPageINdex : ' + thisPageIndex.toString());

    enable = thisPageIndex == pageController?.page?.toInt();

    pageController?.addListener(a);
  }

  void a() {
    if (pageController?.page?.toInt() == thisPageIndex && !enable!) {
      print(thisPageIndex);
      setState(() {
        enable = true;
      });
    } else if (pageController?.page?.toInt() != thisPageIndex && enable!) {
      setState(() {
        enable = false;
      });
    }
  }

  @override
  void dispose() {
    super.dispose();

    pageController?.removeListener(a);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
        onTap: () {
          pageController?.animateToPage(thisPageIndex ?? 0,
              duration: Duration(milliseconds: 500),
              curve: Curves.easeInOutCirc);
        },
        child: Padding(
            padding: const EdgeInsets.symmetric(vertical: 10),
            child: AwareText(
              widget.pageContent?.title,
              builder: (text) {
                return Text(
                  text,
                  style:
                      TextStyle(color: enable! ? Colors.white : Colors.white38),
                );
              },
            )));
  }
}

import 'package:dorvan/application/constants/theme.dart';
import 'package:dorvan/application/use_cases/precache_next_image.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/exit_page.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:dorvan/presentation/article_view.dart';
import 'package:dorvan/presentation/bottom_bar.dart';
import 'package:dorvan/presentation/exit_page.dart';
import 'package:dorvan/presentation/picture_view.dart';
import 'package:flutter/material.dart';

import 'aware_text.dart';

class SecondScrollView extends StatefulWidget {
  final Chapter? chapter;

  SecondScrollView({@required this.chapter});

  @override
  _SecondScrollViewState createState() => _SecondScrollViewState();
}

class _SecondScrollViewState extends State<SecondScrollView> {
  PageController detailPageController = PageController(viewportFraction: 1);

  @override
  void dispose() {
    super.dispose();
    detailPageController.dispose();
  }

  @override
  void initState() {
    super.initState();

    pages = widget.chapter?.pages ?? [];
  }

  List<PageContent> pages = [];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      //backgroundColor: Color(0xfff2f2f2), // grey
      backgroundColor: Color(0xffF6F8F9), // blue
      body: Stack(
        children: [
          /*PageView(
            controller: detailPageController,
            scrollDirection: Axis.vertical,
            children: [
              ...?widget.chapter?.pages.map((page) {
                if (page is Article) {
                  return ArticleView(article: page);
                } else if (page is Picture) {
                  return PictureView(
                    picture: page,
                  );
                } else
                  return SizedBox.shrink();
              }).toList(),
              ExitPage(),
            ],
          ),*/
          PageView.builder(
            controller: detailPageController,
            scrollDirection: Axis.vertical,
            itemCount: pages.length,
            itemBuilder: (context, index) {
              final page = pages[index];

              // Precache image
              if (index + 1 < pages.length) {
                PrecacheNextImage(context, pages[index + 1]);
              }
              if (page is Article) {
                return ArticleView(article: page);
              } else if (page is Picture) {
                return PictureView(
                  picture: page,
                );
              } else if (page is ExitPage) {
                return ExitPageView(
                  exitPage: page,
                );
              } else
                return SizedBox.shrink();
            },
          ),
          Padding(
            padding: EdgeInsets.all(30),
            child: Stack(
              children: [
                // Back arrow
                Align(
                  alignment: Alignment.topLeft,
                  child: GestureDetector(
                    onTap: () {
                      Navigator.of(context).pop();
                    },
                    child: Icon(
                      Icons.arrow_back_rounded,
                      color: kArticleDescriptionColor,
                    ),
                  ),
                ),
                // Scroll bar
                Align(
                  alignment: Alignment.bottomCenter,
                  child: BottomBar(
                    brightness: Brightness.light,
                    pageController: detailPageController,
                    totalPage: pages.length,
                  ),
                )
              ],
            ),
          ),
        ],
      ),
    );
  }
}

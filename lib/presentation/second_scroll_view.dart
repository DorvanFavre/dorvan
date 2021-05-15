import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:dorvan/presentation/article_view.dart';
import 'package:dorvan/presentation/bottom_bar.dart';
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
  PageController detailPageController = PageController(
    viewportFraction: 1
  );

  @override
  void dispose() {
    super.dispose();
    detailPageController.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Color(0xfff2f2f2),
      body: Stack(
        children: [
          PageView.builder(
            controller: detailPageController,
            scrollDirection: Axis.vertical,
            itemCount: widget.chapter?.pages.length,
            itemBuilder: (context, index) {
              final page = widget.chapter?.pages[index];
              if (page is Article) {
                return ArticleView(article: page);
              } else if (page is Picture) {
                return PictureView(
                  picture: page,
                );
              } else
                return SizedBox.shrink();
            },
          ),
          Padding(
            padding: EdgeInsets.all(30),
            child: Stack(
              children: [
                Align(
                  alignment: Alignment.topLeft,
                  child: GestureDetector(
                    onTap: () {
                      Navigator.of(context).pop();
                    },
                    child: Icon(
                      Icons.arrow_back_rounded,
                      color: Colors.grey,
                    ),
                  ),
                ),
                Align(
                  alignment: Alignment.bottomCenter,
                  child: BottomBar(
                    brightness: Brightness.light,
                    pageController: detailPageController,
                    totalPage: widget.chapter!.pages.length,
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

import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/contact.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

final appContentProvider = Provider<List<PageContent>>((ref) {
  return [
    Intro(
      "chapter_intro_title",
      "chapter_intro_description",
      '00',
    ),
    Picture('dorvan.jpg', 'Vans', 'salut'),
    // Entrepreneur
    Chapter('chapter_entrepreneur_title', "chapter_entrepreneur_description",
        '01', [
      Article('article_entrepreneur_title', 'article_entrepreneur_description',
          '01'),
      Picture('02_iphone12.jpg', 'Vans', 'salut'),
      Article('article_parlezmoi_title', 'article_parlezmoi_description', '02'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article(
          'article_tirerprofit_title', 'article_tirerprofit_description', '03'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article(
          'article_boutenbout_title', 'article_boutenbout_description', '04'),
      Picture('vans.jpg', 'Vans', 'salut'),
    ]),
    Picture('02_iphone12.jpg', 'Vans', 'salut'),
    // Architect
    Chapter('chapter_architect_title', "chapter_architect_description", '05', [
      Article('article_architect_title', 'article_architect_description', '05'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('article_nature_title', 'article_nature_description', '06'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('article_pwa_title', 'article_pwa_description', '07'),
      Picture('vans.jpg', 'Vans', 'salut'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Chapter('chapter_designer_title', "chapter_designer_description", '08', [
      Article('article_designer_title', 'article_designer_description', '08'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('article_artetweb_title', 'article_artetweb_description', '09'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('article_premiereimpression_title',
          'article_premiereimpression_description', '10'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article(
          'article_matechnique_title', 'article_matechnique_description', '11'),
      Picture('vans.jpg', 'Vans', 'salut'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Chapter(
        'chapitre_developer_title', "chapitre_developer_description", '12', [
      Article('article_developer_title', 'article_developer_description', '12'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article(
          'article_technologie_title', 'article_technologie_description', '13'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('article_language_title', 'article_language_description', '14'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article(
          'article_monsiteweb_title', 'article_monsiteweb_description', '15'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Contact('Contact'),
    Picture('vans.jpg', 'Vans', 'salut'),
  ];
});
